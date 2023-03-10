# Company.Node1

Edit `ros2cs.spec` to include ROS 2 interface packages you wish to use.

Install `ros2cs` utility with the following command if not yet installed:
```
dotnet tool install -g ros2cs
```
After installation, simply run `ros2cs` in the project root to generate messages.

This project can also be built as a colcon package. Feel free to modify `package.xml`
and `CMakeLists.txt` to specify package dependencies and add other resources
you would like to include in the package.

See https://github.com/noelex/rclnet for more information.