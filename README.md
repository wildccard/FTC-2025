# FTC-2025 Robot Code

Subsystem-oriented FTC Java template for your 4-wheel swerve + intake/sorter/turret/shooter robot.

## Structure
- `SwerveDriveSubsystem` + `SwerveModule`: 4-wheel field-centric swerve using continuous steering servos and REV Through Bore analog angle feedback.
- `IntakeSubsystem`: dual goBILDA 5202 intake motors.
- `SortingSubsystem`: 3 slots, 2 color sensors per slot, 1 shovel servo per slot.
- `VisionSubsystem`: Limelight A1 AprilTag mapping.
- `TurretSubsystem`: dual continuous-servos with through-bore relative encoder and heading compensation.
- `ShooterSubsystem`: dual flywheel motors + hood angle servo.
- `RobotContainer`: hardware binding shared by TeleOp and Auto.
- `MainTeleOp` and `AutoSortShoot`: example op modes.

## AprilTag rules encoded
- Sorting pattern:
  - ID 21 -> Green, Purple, Purple
  - ID 22 -> Purple, Green, Purple
  - ID 23 -> Purple, Purple, Green
- Goal tags:
  - ID 20 -> Blue goal
  - ID 24 -> Red goal

## HardwareMap names expected
### Swerve
- `fl_drive`, `fr_drive`, `bl_drive`, `br_drive`
- `fl_steer`, `fr_steer`, `bl_steer`, `br_steer`
- `fl_encoder`, `fr_encoder`, `bl_encoder`, `br_encoder`

### Intake / sorter / shooter / turret
- `intake_left`, `intake_right`
- `slot1_a`, `slot1_b`, `slot2_a`, `slot2_b`, `slot3_a`, `slot3_b`
- `shovel1`, `shovel2`, `shovel3`
- `flywheel_left`, `flywheel_right`, `hood`
- `turret_left`, `turret_right`, `turret_encoder`

## Important integration TODOs
- In `VisionSubsystem`, replace placeholder methods with your Limelight FTC SDK calls.
- In `PinpointHeadingProvider`, return heading from your Pinpoint localization system.
- Tune PID constants and servo/motor directions for your robot.
