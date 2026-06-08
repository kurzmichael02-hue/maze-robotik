# Meeting notes

Two fixed slots a week, mondays and thursdays, ~45 min each. Team: Michael Kurz,
Schayan Goudarzi (both there by default, we're only two). ROS 2 base image from the lecturer.

| Date | Discussed | Todos until next time |
|---|---|---|
| Mon 13.04 · kickoff | Scoped it: robot navigates the maze on its own, finds the shortest path. Stack ROS 2 Jazzy + Gazebo Harmonic + slam_toolbox. Rough split: Schayan world/robot/generator, Michael planners/benchmark. | **M:** repo + docker on the lecturer's ros2 image · **S:** read up on procedural maze generation |
| Thu 16.04 | Docker runs for both (vcxsrv on windows). Decided to bake the bot into the world SDF, saves the spawn step. | **M:** grid representation (cells + walls) · **S:** recursive-backtracker generator, seed-deterministic |
| Mon 20.04 | First mazes look good, bot URDF with 360° lidar up. Difficulty via share of extra passages. | **M:** BFS + A* on the grid, shared interface · **S:** start/goal markers + difficulty levels |
| Thu 23.04 | ros<->gz bridge up, lidar arrives in ROS. slam first runs, still shaky. | **M:** tune slam_async.yaml (resolution, laser range) · **S:** multi-path mode in the generator |
| Mon 27.04 | Controller settled: deterministic path_executor (cell by cell), frontier_explorer as an extra for pure slam exploration. | **M:** finish path_executor · **S:** document difficulty numbers (walls per level) |
| Thu 30.04 · cancelled | Michael sick, quick sync over chat, nothing blocking. | pushed to monday |
| Mon 04.05 | slam dialed in (5 cm grid, loop closure on), map comes out clean. Planner base + BFS + A* run on test grids. | **M:** add wall-follower + flood-fill behind one interface · **S:** generate/test worlds with difficulty levels |
| Thu 07.05 | All four algos done. Idea: a `/run_planners` service that runs all four on the current slam map. | **M:** planner_node + map_converter (occupancy grid → cells) · **S:** maze textures, still grey |
| Mon 11.05 | Service works, paths come out as nav_msgs/Path. Agreed metrics: length, expansions, turns, runtime. | **M:** benchmark.py sweeping sizes + seeds · **S:** rviz path markers |
| Thu 14.05 | Benchmark over 5–20 mazes, A* clearly fewer expansions than BFS. | **M:** write out plots (docs/benchmark) · **S:** rviz config for colored path lines |
| Mon 18.05 | Paths as colored lines in rviz, looks strong. Noticed wall-follower loops on non-perfect mazes (islands). | **M:** analyze the behavior + explain in README · **together:** full end-to-end run |
| Thu 21.05 | Benchmark pushed to 25×25, numbers consistent (all smart algos 200). | **M:** final benchmark CSV + plot into docs · **S:** textures done, start/goal clearer |
| Mon 25.05 · short | Quick sync, both tracks moving on the side, no blockers. | next: docs + presentation planning |
| Thu 28.05 | README structure set: architecture diagram, components, usage, numbers. | **M:** write README + embed benchmark plot · **S:** screenshots/docs for maze_worlds |
| Mon 01.06 | Full integration test: bot drives through on its own, reaches goal after ~2 min, slam maps cleanly. | **together:** prepare presentation · **M:** cleanup, dead prints out, tests green |
| Thu 04.06 | Presentation walkthrough, who shows what. Live demo stable. Small fixes (execution-policy note, container reset tip). | finish the submission |
| Mon 08.06 · wrap-up | Submission out. Retro: task split worked well, slam tuning ate the most time, the algo comparison landed really well. | result later: **96/100** |
