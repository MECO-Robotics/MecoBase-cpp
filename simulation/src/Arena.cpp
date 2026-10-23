#include "base/sim/Arena.h"
#include <cstdlib>
#include <string_view>
#include <algorithm>
#include <cmath>
#include <random>
#include <stdexcept>

namespace base::sim
{
namespace
{
Vec3 Add(Vec3 a, Vec3 b)
{
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}
Vec3 Cross(Vec3 a, Vec3 b)
{
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
Vec3 Rotate(Quaternion q, Vec3 v)
{
    Vec3 u{q.x, q.y, q.z};
    auto t = Cross(u, v);
    t = {2 * t.x, 2 * t.y, 2 * t.z};
    auto c = Cross(u, t);
    return {v.x + q.w * t.x + c.x, v.y + q.w * t.y + c.y, v.z + q.w * t.z + c.z};
}
Quaternion Mul(Quaternion a, Quaternion b)
{
    return {a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y, a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w, a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
}
Quaternion Inverse(Quaternion q)
{
    return {-q.x, -q.y, -q.z, q.w};
}
void ValidateIntake(const IntakeConfig &i)
{
    auto q = i.localPose.rotation;
    double n = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    if (!std::isfinite(i.localPose.position.x) || !std::isfinite(i.localPose.position.y) ||
        !std::isfinite(i.localPose.position.z) || !std::isfinite(n) || std::abs(n - 1) > 1e-5 ||
        !std::isfinite(i.halfExtents.x) || !std::isfinite(i.halfExtents.y) || !std::isfinite(i.halfExtents.z) ||
        i.halfExtents.x <= 0 || i.halfExtents.y <= 0 || i.halfExtents.z <= 0)
        throw std::invalid_argument("Invalid intake geometry");
}
SimulationWorld &RequireWorld(SimulationClock &clock)
{
    if (!clock.World())
        throw std::invalid_argument("Arena requires Jolt backend");
    return *clock.World();
}
bool Finite(Vec3 v)
{
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}
} // namespace
Arena::Arena(SimulationClock &clock, std::function<BodyState()> truth, bool scene, uint32_t seed,
             Vec3 robotCenterOfMass)
    : Arena(clock, std::move(truth), scene ? ArenaScene::Generic : ArenaScene::Empty, seed, robotCenterOfMass)
{
}
Arena::Arena(SimulationClock &clock, std::function<BodyState()> truth, ArenaScene scene, uint32_t seed,
             Vec3 robotCenterOfMass)
    : clock_(clock), world_(RequireWorld(clock)), robotTruth_(std::move(truth)), robotCenterOfMass_(robotCenterOfMass),
      scene_(scene), seed_(seed)
{
    if (!robotTruth_)
        throw std::invalid_argument("Arena requires robot truth");
    if (!Finite(robotCenterOfMass_))
        throw std::invalid_argument("Invalid robot center of mass");
    try
    {
        if (scene == ArenaScene::Generic)
            BuildScene();
        Reset(seed);
        registration_ = clock.Register(
            {[this](double dt) { Before(dt); }, [this](double dt) { After(dt); }, [this] { Reset(seed_); }});
    }
    catch (...)
    {
        ClearPieces();
        for (auto b : colliders_)
            world_.DestroyBody(b.body);
        throw;
    }
}
Arena::~Arena()
{
    registration_ = Registration{};
    ClearPieces();
    for (auto b : colliders_)
        world_.DestroyBody(b.body);
}
void Arena::ClearPieces()
{
    for (auto &p : pieces_)
        if (p.body)
            world_.DestroyBody(*p.body);
    pieces_.clear();
    for (auto &i : intakes_)
        i.held.clear();
}
void Arena::AddCollider(const BodySpec &spec)
{
    if (spec.mass != 0)
        throw std::invalid_argument("Arena collider must be static");
    auto body = world_.CreateBody(spec);
    try
    {
        colliders_.push_back({body, spec});
    }
    catch (...)
    {
        world_.DestroyBody(body);
        throw;
    }
}
std::vector<BodySpec> Arena::ColliderGeometry() const
{
    std::vector<BodySpec> geometry;
    geometry.reserve(colliders_.size());
    for (const auto &collider : colliders_)
        geometry.push_back(collider.geometry);
    return geometry;
}
BodyHandle Arena::CreatePiece(const PieceSpec &p, bool airborne, const BodyState *state)
{
    BodySpec s;
    s.shape = p.shape;
    s.radius = p.radius;
    s.halfExtents = p.halfExtents;
    s.mass = p.mass;
    s.friction = p.friction;
    s.restitution = p.restitution;
    s.pose = p.pose;
    s.planar = world_.Mode() == Backend::Planar && !airborne;
    auto body = world_.CreateBody(s);
    try
    {
        BodyState value = state ? *state : BodyState{p.pose, p.velocity, {}, true};
        world_.SetBodyState(body, value);
    }
    catch (...)
    {
        world_.DestroyBody(body);
        throw;
    }
    return body;
}
PieceId Arena::Spawn(const PieceSpec &p)
{
    if (!std::isfinite(p.mass) || p.mass <= 0 || !std::isfinite(p.rollingResistance) || p.rollingResistance < 0 ||
        !Finite(p.velocity))
        throw std::invalid_argument("Invalid piece motion");
    auto body = CreatePiece(p, p.airborne);
    auto id = nextId_++;
    try
    {
        pieces_.push_back({id, p, body, p.airborne, 0});
    }
    catch (...)
    {
        world_.DestroyBody(body);
        throw;
    }
    return id;
}
void Arena::SetIntake(const IntakeConfig &i)
{
    ValidateIntake(i);
    intakes_[0].config = i;
}
IntakeId Arena::RegisterIntake(const IntakeConfig &i)
{
    ValidateIntake(i);
    intakes_.push_back({i, false, {}});
    return intakes_.size() - 1;
}
void Arena::SetIntakeActive(IntakeId id, bool active)
{
    intakes_.at(id).active = active;
}
bool Arena::IntakeActive(IntakeId id) const
{
    return intakes_.at(id).active;
}
size_t Arena::HeldCount(IntakeId id) const
{
    return intakes_.at(id).held.size();
}
std::optional<PieceId> Arena::Launch(IntakeId intakeId, Pose muzzle, Vec3 velocity)
{
    auto &held_ = intakes_.at(intakeId).held;
    if (!Finite(velocity))
        throw std::invalid_argument("Invalid launch velocity");
    if (held_.empty())
        return {};
    auto id = held_.front();
    auto it = std::find_if(pieces_.begin(), pieces_.end(), [id](const Piece &p) { return p.id == id; });
    auto robot = robotTruth_();
    auto offset = Rotate(robot.pose.rotation, muzzle.position);
    auto com = Rotate(robot.pose.rotation, robotCenterOfMass_);
    Vec3 lever{offset.x - com.x, offset.y - com.y, offset.z - com.z};
    BodyState state;
    state.pose = {Add(robot.pose.position, offset), Mul(robot.pose.rotation, muzzle.rotation)};
    state.linearVelocity =
        Add(Add(robot.linearVelocity, Cross(robot.angularVelocity, lever)), Rotate(state.pose.rotation, velocity));
    auto body = CreatePiece(it->spec, true, &state);
    it->body = body;
    it->airborne = true;
    it->settled = 0;
    held_.pop_front();
    return id;
}
std::vector<PieceSnapshot> Arena::Pieces() const
{
    std::vector<PieceSnapshot> out;
    out.reserve(pieces_.size());
    for (const auto &p : pieces_)
    {
        BodyState state = p.body ? world_.ReadBody(*p.body) : robotTruth_();
        state.enabled = p.body.has_value();
        out.push_back({p.id, state, !p.body, p.airborne});
    }
    return out;
}
void Arena::Before(double dt)
{
    for (auto &p : pieces_)
        if (p.body && !p.airborne)
        {
            auto s = world_.ReadBody(*p.body);
            double speed = std::hypot(s.linearVelocity.x, s.linearVelocity.y);
            if (speed > 0)
            {
                double scale = std::max(0.0, speed - p.spec.rollingResistance * dt) / speed;
                s.linearVelocity.x *= scale;
                s.linearVelocity.y *= scale;
                world_.SetLinearVelocity(*p.body, s.linearVelocity);
            }
        }
}
void Arena::After(double dt)
{
    auto robot = robotTruth_();
    for (auto &p : pieces_)
        if (p.body)
        {
            auto state = world_.ReadBody(*p.body);
            bool captured = false;
            for (auto &intake : intakes_)
            {
                auto &intake_ = intake.config;
                auto &held_ = intake.held;
                auto iq = Mul(robot.pose.rotation, intake_.localPose.rotation);
                auto center = Add(robot.pose.position, Rotate(robot.pose.rotation, intake_.localPose.position));
                auto delta = Rotate(Inverse(iq), {state.pose.position.x - center.x, state.pose.position.y - center.y,
                                                  state.pose.position.z - center.z});
                if (intake.active && held_.size() < intake_.capacity && std::abs(delta.x) <= intake_.halfExtents.x &&
                    std::abs(delta.y) <= intake_.halfExtents.y && std::abs(delta.z) <= intake_.halfExtents.z)
                {
                    held_.push_back(p.id);
                    world_.DestroyBody(*p.body);
                    p.body.reset();
                    captured = true;
                    break;
                }
            }
            if (captured)
                continue;
            double height = p.spec.shape == BodySpec::Shape::Sphere ? p.spec.radius : p.spec.halfExtents.z;
            bool upright = p.spec.shape == BodySpec::Shape::Sphere ||
                           (std::abs(state.pose.rotation.x) < 0.05 && std::abs(state.pose.rotation.y) < 0.05);
            bool settled = state.pose.position.z <= height + 0.025 && state.pose.position.z >= height - 0.025 &&
                           std::abs(state.linearVelocity.z) < 0.12 &&
                           (p.spec.shape == BodySpec::Shape::Sphere ||
                            std::hypot(state.angularVelocity.x, state.angularVelocity.y) < 0.15) &&
                           upright;
            p.settled = settled ? p.settled + dt : 0;
            if (p.airborne && p.settled >= 0.2)
            {
                if (world_.Mode() == Backend::Planar)
                {
                    state.pose.position.z = height;
                    state.linearVelocity.z = 0;
                    state.angularVelocity.x = state.angularVelocity.y = 0;
                    // Create first: failure preserves the original body's ownership.
                    auto body = CreatePiece(p.spec, false, &state);
                    world_.DestroyBody(*p.body);
                    p.body = body;
                }
                p.airborne = false;
            }
        }
}
void Arena::BuildScene()
{
    auto box = [this](Vec3 position, Vec3 half, Quaternion rotation = Quaternion{}) {
        BodySpec b;
        b.pose = {position, rotation};
        b.halfExtents = half;
        AddCollider(b);
    };
    const auto center = kPracticeStart;
    box({center.x, center.y, -0.15}, {kPracticeLength / 2 + .2, kPracticeWidth / 2 + .2, .15});
    box({-.1, center.y, .5}, {.1, kPracticeWidth / 2 + .2, .5});
    box({kPracticeLength + .1, center.y, .5}, {.1, kPracticeWidth / 2 + .2, .5});
    box({center.x, -.1, .5}, {kPracticeLength / 2, .1, .5});
    box({center.x, kPracticeWidth + .1, .5}, {kPracticeLength / 2, .1, .5});
    box({center.x + 3, center.y + 2.5, .4}, {.6, .5, .4});
    if (world_.Mode() == Backend::Full3D)
        box({center.x - 3, center.y + 2, .2}, {1, .7, .1}, {0, std::sin(-0.1), 0, std::cos(-0.1)});
}
void Arena::Reset(uint32_t seed)
{
    ClearPieces();
    seed_ = seed;
    for (auto &i : intakes_)
        i.active = false;
    nextId_ = 1;
    if (scene_ == ArenaScene::Empty)
        return;
    std::mt19937 random(seed);
    std::uniform_real_distribution<double> x(kPracticeStart.x - 5, kPracticeStart.x + 5),
        y(kPracticeStart.y + 1.3, kPracticeStart.y + 3.3);
    PieceSpec p;
    p.pose.position = {kPracticeStart.x + .7, kPracticeStart.y, .12};
    Spawn(p);
    for (int i = 0; i < 8; ++i)
    {
        p.pose.position = {x(random), y(random), 0.12};
        Spawn(p);
    }
}
} // namespace base::sim
