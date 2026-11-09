#ifndef __FRC_ROBORIO__
#include "base/NtGamepad.h"
#include <gtest/gtest.h>
#include <array>
#include <limits>

TEST(NtGamepad, RejectsMalformedOrOutOfRangeAxes) {
  std::array<double,6> packet{1,1,1000000,.4,0,0};
  EXPECT_TRUE(base::NtGamepad::ValidPayload(packet));
  packet[3]=1.01;
  EXPECT_FALSE(base::NtGamepad::ValidPayload(packet));
  packet[3]=std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(base::NtGamepad::ValidPayload(packet));
  packet[3]=0;packet[1]=1.5;
  EXPECT_FALSE(base::NtGamepad::ValidPayload(packet));
}
TEST(NtGamepad, HoldsFreshSampleThenNeutralizesStaleAndOutOfOrderInput) {
  auto instance=nt::NetworkTableInstance::Create();
  {
    base::NtGamepad input{instance};
    auto publisher=instance.GetDoubleArrayTopic("/Input/Gamepad").Publish();
    const auto now=nt::Now();
    std::array<double,6> packet{1,2,static_cast<double>(now),.4,0,0};
    publisher.Set(packet,now);
    EXPECT_TRUE(input.Read(now+1000).accepted);
    EXPECT_DOUBLE_EQ(input.Read(now+2000).leftY,.4);
    auto stale=input.Read(now+51000);
    EXPECT_FALSE(stale.accepted);
    EXPECT_DOUBLE_EQ(stale.leftY,0);
    packet[1]=1;packet[2]=static_cast<double>(now+52000);
    publisher.Set(packet,now+52000);
    EXPECT_FALSE(input.Read(now+53000).accepted);
    packet[1]=3;packet[2]=static_cast<double>(now+54000);packet[3]=-.4;
    publisher.Set(packet,now+54000);
    EXPECT_TRUE(input.Read(now+55000).accepted);
    EXPECT_DOUBLE_EQ(input.Read(now+56000).leftY,-.4);
    // A delayed source must expire by its own timestamp, even when the NT
    // publication metadata is newer and the cached packet is read again.
    packet[1]=4;packet[2]=static_cast<double>(now+45000);
    publisher.Set(packet,now+60000);
    EXPECT_TRUE(input.Read(now+61000).accepted);
    EXPECT_FALSE(input.Read(now+96000).accepted);
  }
  nt::NetworkTableInstance::Destroy(instance);
}
#endif

#ifndef __FRC_ROBORIO__
#include "base/DriverInput.h"
#include <frc/simulation/DriverStationSim.h>
#include <hal/HAL.h>

TEST(DriverInput, SelectsFreshInputAndEnforcesTeleopAuthority) {
  HAL_Initialize(500, 0);
  frc::sim::DriverStationSim::ResetData();
  frc::sim::DriverStationSim::SetDsAttached(true);
  frc::sim::DriverStationSim::SetJoystickAxisCount(0, 6);
  frc::sim::DriverStationSim::SetJoystickAxis(0, 1, .3);
  frc::sim::DriverStationSim::SetEnabled(true);
  frc::sim::DriverStationSim::SetAutonomous(false);
  frc::sim::DriverStationSim::NotifyNewData();
  frc::DriverStation::RefreshData();
  frc::XboxController controller{0};
  auto instance = nt::NetworkTableInstance::Create();
  {
    base::NtGamepad ntInput{instance};
    auto ds = base::ReadDriverInput(controller, &ntInput, true);
    EXPECT_EQ(ds.source, "driverstation");
    EXPECT_TRUE(ds.authorized);
    EXPECT_NEAR(ds.axes.leftY, .3, .01);
    EXPECT_FALSE(base::ReadDriverInput(controller, &ntInput, false).available);
    auto publisher = instance.GetDoubleArrayTopic("/Input/Gamepad").Publish();
    const auto now = nt::Now();
    std::array<double, 6> sample{1, 1, static_cast<double>(now), -.7, 0, 0};
    publisher.Set(sample, now);
    auto nt = base::ReadDriverInput(controller, &ntInput, true);
    EXPECT_EQ(nt.source, "nt");
    EXPECT_DOUBLE_EQ(nt.axes.leftY, -.7);
    frc::sim::DriverStationSim::SetEnabled(false);
    frc::sim::DriverStationSim::NotifyNewData();
    frc::DriverStation::RefreshData();
    auto disabled = base::ReadDriverInput(controller, &ntInput, true);
    EXPECT_FALSE(disabled.authorized);
    EXPECT_DOUBLE_EQ(disabled.axes.leftY, 0);
  }
  nt::NetworkTableInstance::Destroy(instance);
  frc::sim::DriverStationSim::ResetData();
  frc::sim::DriverStationSim::NotifyNewData();
  frc::DriverStation::RefreshData();
}
#endif
