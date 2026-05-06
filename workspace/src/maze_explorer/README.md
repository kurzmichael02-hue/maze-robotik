# maze_explorer

selbstständige erkundung des labyrinths via frontier-detection + slam.

## todo
- [ ] slam_toolbox config tunen für maze (lidar range begrenzen)
- [ ] frontier-detection node (occupancy grid -> frontier centroids)
- [ ] goal-selection: nächste/größte/beste frontier
- [ ] anbindung an nav2 action client
- [ ] "fertig"-erkennung wenn keine frontiers mehr da sind

## konzept

```
/scan (lidar) -> slam_toolbox -> /map (occupancy grid)
                                    |
                                    v
                            frontier_detector
                                    |
                                    v
                    pick goal (nearest / largest / info-gain)
                                    |
                                    v
                            nav2 action -> /cmd_vel
```
