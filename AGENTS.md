# AGENTS.md

## README Changes

- Do not modify `README.md` without the user's explicit instruction.

## ROS Integration Testing

- Use the Visual Studio Docker test environments defined in `src/testEnvironments.json`.
- Reuse existing `rclnet-tests:<distro>` images and Docker build cache. Rebuild only when the corresponding Dockerfile or test interface package changes.
- Test images include the .NET 10 SDK and the .NET 8/9 runtimes. Run tests inside the reusable container so the ROS and .NET environments match.
- Do not remove reusable `rclnet-tests:*` images as routine cleanup.
- If the Visual Studio test environment cannot be used, run `dotnet test` inside the corresponding reusable test image instead of introducing another test runner.
