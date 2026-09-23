# AGENTS.md

## ROS Integration Testing

- Use the Visual Studio Docker test environments defined in `src/testEnvironments.json`.
- Reuse existing `rclnet-tests:<distro>` images and Docker build cache. Rebuild only when the corresponding Dockerfile or test interface package changes.
- Keep test images runtime-only. Build test assemblies with the host SDK or Visual Studio; do not install or download a .NET SDK inside test containers.
- Do not remove reusable `rclnet-tests:*` images as routine cleanup.
- If the Visual Studio test environment cannot be used, report the limitation before introducing another test runner or downloading an SDK.
