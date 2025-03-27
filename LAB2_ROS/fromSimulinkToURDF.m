robot = import("DoublePendulum_fromSim2URDF.slx")
exporter = urdfExporter(robot)
exporter.OutputFileName = "DoublePendulum_exported.urdf"
exporter.RobotName = "DoublePendulum"
exporter.writefile()
