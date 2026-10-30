#include "base/ObjectPerception.h"
#include <frc/Timer.h>
#include <frc/interpolation/TimeInterpolatableBuffer.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/NetworkTableInstance.h>
#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>

namespace base {
namespace {
constexpr size_t kMaximumDetections = 128;
void ValidateConfig(const ObjectPerceptionConfig &config) {
    if (!std::isfinite(config.freshness.value()) || config.freshness.value()<=0 || config.freshness.value()>1 ||
        !std::isfinite(config.maxDistance.value()) || config.maxDistance.value()<=0 ||
        !std::isfinite(config.clusterRadius.value()) || config.clusterRadius.value()<=0 ||
        !std::isfinite(config.minimumConfidence) || !(config.minimumConfidence>0 && config.minimumConfidence<=1))
        throw std::invalid_argument("Invalid object perception configuration");
}
bool Finite(const frc::Translation3d& p) {
    return std::isfinite(p.X().value()) && std::isfinite(p.Y().value()) && std::isfinite(p.Z().value());
}
class NtDetectionIO final : public ObjectDetectionIO {
 public:
    explicit NtDetectionIO(std::string topic) : input_(nt::NetworkTableInstance::GetDefault().GetDoubleArrayTopic(topic).Subscribe({})) {}
    std::optional<ObjectDetectionFrame> ReadFrame() override {
        const auto sample=input_.GetAtomic();
        if (!sample.time || sample.time==previous_) return std::nullopt;
        previous_=sample.time;
        const auto& v=sample.value;
        if (v.size()<3 || (v.size()-3)%4 || (v.size()-3)/4>kMaximumDetections ||
            !std::all_of(v.begin(),v.end(),[](double x){return std::isfinite(x);}) ||
            v[0]!=1 || v[1]<0 || v[1]>9007199254740991. || std::floor(v[1])!=v[1] || v[2]<0) return std::nullopt;
        const double age=(nt::Now()-sample.time)*1e-6;
        if (age<0) return std::nullopt;
        ObjectDetectionFrame frame{static_cast<uint64_t>(v[1]),frc::Timer::GetFPGATimestamp()-units::second_t{age+v[2]}, {}};
        for (size_t i=3;i<v.size();i+=4) {
            if (v[i+3]<0 || v[i+3]>1) return std::nullopt;
            frame.detections.push_back({{units::meter_t{v[i]},units::meter_t{v[i+1]},units::meter_t{v[i+2]}},v[i+3]});
        }
        return frame;
    }
 private:
    nt::DoubleArraySubscriber input_;
    int64_t previous_{};
};
}
std::unique_ptr<ObjectDetectionIO> MakeNtObjectDetectionIO(std::string topic) {
    if (topic.empty() || topic.front()!='/') throw std::invalid_argument("Object detection topic must be absolute");
    return std::make_unique<NtDetectionIO>(std::move(topic));
}
struct ObjectPerception::State {
    struct Observed { frc::Translation2d point; double confidence; units::second_t time; };
    struct Camera {
        explicit Camera(ObjectCamera value) : config(std::move(value)), mounts(units::second_t{1}) {}
        ObjectCamera config;
        frc::TimeInterpolatableBuffer<frc::Pose3d> mounts;
        std::vector<Observed> points;
        std::optional<uint64_t> sequence;
    };
    std::vector<Camera> cameras;
    std::function<frc::Pose3d()> robotPose;
    ObjectPerceptionConfig config;
    frc::TimeInterpolatableBuffer<frc::Pose3d> poses{units::second_t{1}};
    std::vector<ObjectCluster> clusters;
};
ObjectPerception::ObjectPerception(std::vector<ObjectCamera> cameras, std::function<frc::Pose3d()> robotPose,
        ObjectPerceptionConfig config) : state_(std::make_unique<State>()) {
    if (!robotPose) throw std::invalid_argument("Object perception requires a robot pose supplier");
    ValidateConfig(config);
    state_->robotPose=std::move(robotPose);state_->config=config;
    for (auto& camera:cameras) {
        if (!camera.io || !camera.robotToCamera) throw std::invalid_argument("Camera requires IO and mounting supplier");
        state_->cameras.emplace_back(std::move(camera));
    }
}
ObjectPerception::~ObjectPerception()=default;
const std::vector<ObjectCluster>& ObjectPerception::Clusters() const { return state_->clusters; }
void ObjectPerception::Clear() {
    state_->poses.Clear();state_->clusters.clear();
    for (auto& camera:state_->cameras) {camera.mounts.Clear();camera.points.clear();}
}
void ObjectPerception::SetConfig(const ObjectPerceptionConfig &config) {
    ValidateConfig(config);
    state_->config=config;
    Clear();
}
void ObjectPerception::Periodic() { UpdateAt(frc::Timer::GetFPGATimestamp(),state_->robotPose()); }
void ObjectPerception::UpdateAt(units::second_t now, const frc::Pose3d& pose) {
    auto& s=*state_;
    if (!std::isfinite(now.value()) || !Finite(pose.Translation())) { Clear();return; }
    s.poses.AddSample(now,pose);
    std::vector<State::Observed> points;
    for (auto& camera:s.cameras) {
        const auto mount=camera.config.robotToCamera();
        if (!mount || !Finite(mount->Translation())) {camera.points.clear();camera.mounts.Clear();continue;}
        camera.mounts.AddSample(now,frc::Pose3d{mount->Translation(),mount->Rotation()});
        const auto frame=camera.config.io->ReadFrame();
        if (frame && (!camera.sequence || frame->sequence!=*camera.sequence)) {
            camera.sequence=frame->sequence;
            const double age=(now-frame->captureTime).value();
            const auto& poseHistory=s.poses.GetInternalBuffer();
            const auto& mountHistory=camera.mounts.GetInternalBuffer();
            if (std::isfinite(age) && age>=0 && age<=s.config.freshness.value() &&
                frame->detections.size()<=kMaximumDetections && frame->captureTime>=poseHistory.front().first &&
                frame->captureTime>=mountHistory.front().first) {
                const auto chassis=s.poses.Sample(frame->captureTime);
                const auto oldMount=camera.mounts.Sample(frame->captureTime);
                const auto optical=chassis->TransformBy(frc::Transform3d{oldMount->Translation(),oldMount->Rotation()});
                std::vector<State::Observed> observed;
                for (const auto& detection:frame->detections) {
                    const auto& p=detection.cameraPoint;
                    if (!Finite(p) || p.X().value()<=0 || p.Norm()>s.config.maxDistance ||
                        !std::isfinite(detection.confidence) || detection.confidence<s.config.minimumConfidence || detection.confidence>1) continue;
                    const auto field=optical.TransformBy(frc::Transform3d{p,frc::Rotation3d{}}).Translation();
                    observed.push_back({field.ToTranslation2d(),detection.confidence,frame->captureTime});
                }
                // A missed frame does not invalidate an observation sooner than
                // its configured freshness deadline. It is never refreshed by
                // empty or invalid frames.
                if (!observed.empty()) camera.points=std::move(observed);
            }
        }
        for (const auto& p:camera.points)
            if (now>=p.time && now-p.time<=s.config.freshness) points.push_back(p);
    }
    std::vector<size_t> parent(points.size());std::iota(parent.begin(),parent.end(),0);
    auto root=[&](size_t i) {while(parent[i]!=i){parent[i]=parent[parent[i]];i=parent[i];}return i;};
    for (size_t i=0;i<points.size();++i) for(size_t j=0;j<i;++j)
        if (points[i].point.Distance(points[j].point)<=s.config.clusterRadius) parent[root(i)]=root(j);
    struct Sum {double x{},y{},weight{};size_t count{};units::second_t time{};};
    std::vector<Sum> sums(points.size());
    for(size_t i=0;i<points.size();++i) {
        auto& sum=sums[root(i)];const auto& p=points[i];
        sum.x+=p.point.X().value()*p.confidence;sum.y+=p.point.Y().value()*p.confidence;
        sum.weight+=p.confidence;++sum.count;sum.time=std::max(sum.time,p.time);
    }
    s.clusters.clear();
    for(const auto& sum:sums) if(sum.count)
        s.clusters.push_back({{units::meter_t{sum.x/sum.weight},units::meter_t{sum.y/sum.weight}},
            sum.count,sum.weight/sum.count,sum.time});
}
}
