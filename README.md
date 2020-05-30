# volaly
[Volaly](http://volaly.ch) (pronounced ['volali]) is a technology for pointing-based human-robot interaction developed by the IDSIA's Robotics Group.

The repository is a ROS-meta package that defines dependencies of the project.

## Dependencies

 - [volaly_bags](https://github.com/idsia-robotics/volaly_bags.git) A simple helper ROS package for recording and storing experimental data in one place.
 - [volaly_kinematics](https://github.com/idsia-robotics/volaly_kinematics.git) Human pointing models and virtual workspace shapes for 3D control.
 - [volaly_localization](https://github.com/idsia-robotics/volaly_localization.git) Pointing-based localization from motion.
 - [volaly_msgs](https://github.com/idsia-robotics/volaly_msgs.git) Common messages used by other Volaly ROS packages.
 - [volaly_robots](https://github.com/idsia-robotics/volaly_robots.git) An interface package for various robots used in experiments.
 - [volaly_supervisor](https://github.com/idsia-robotics/volaly_supervisor.git) A set of finite state machines (FSMs) that drive interaction in experiments.
 
*Note that the methods implemented in `volaly_kinematics` and `volaly_localization` are based on a patent-pending technology, and thus, you have to acquire the license from [IDSIA](http://idsia.ch) to use it in commercial applications.*
 
## Pointing-based interaction

### 2D control
The following video gives an overview of pointing-based interaction and its application to flying and ground robots control. In particular, it demonstrates and briefly describes how does the pointing-based localization from motion method work (implemented in `volaly_localization`).

<p align="center">
  <a href="https://youtu.be/yafy-HZMk_U" alt="Click to play">
    <img src="https://img.youtube.com/vi/yafy-HZMk_U/maxresdefault.jpg" width=70%>
  </a>
</p>

More details available at: http://people.idsia.ch/~gromov/proximity-hri-pipeline/

### Free 3D space
The next video demonstrates the new method that allows one to control quadrotors also in free 3D space (implemented in `volaly_kinematics`).

<p align="center">
  <a href="https://youtu.be/lFksQfQ8rAk" alt="Click to play">
    <img src="https://img.youtube.com/vi/lFksQfQ8rAk/maxresdefault.jpg" width=70%>
  </a>
</p>

More details available at: http://people.idsia.ch/~gromov/3d-control

## Authors

[Boris Gromov](http://people.idsia.ch/~gromov), [Jérôme Guzzi](https://github.com/jeguzzi), and [Alessandro Giusti](http://people.idsia.ch/~giusti).

## Publications

 - B. Gromov, J. Guzzi, L. Gambardella, and A. Giusti, “Intuitive 3D Control of a Quadrotor in User Proximity with Pointing Gestures,” in 2020 IEEE International Conference on Robotics and Automation (ICRA), 2020, to appear.
   [Details](http://people.idsia.ch/~gromov/bibliography/gromov2020intuitive.html).
    <details>
      <summary>Bibtex</summary>

      ```bibtex
      @inproceedings{gromov2020intuitive,
        author = {Gromov, Boris and Guzzi, J{\'e}r{\^o}me and Gambardella, Luca and Giusti, Alessandro},
        title = {Intuitive 3D Control of a Quadrotor in User Proximity with Pointing Gestures},
        booktitle = {2020 IEEE International Conference on Robotics and Automation (ICRA)},
        year = {2020},
        month = may,
        note = {to appear},
      }
      ```
  
    </details>

 - B. Gromov, G. Abbate, L. Gambardella, and A. Giusti, “Proximity Human-Robot Interaction Using Pointing Gestures and a Wrist-mounted IMU,” in 2019 IEEE International Conference on Robotics and Automation (ICRA), 2019, pp. 8084–8091.
   [Details](http://people.idsia.ch/~gromov/bibliography/gromov2019proximity.html).
    <details>
      <summary>Bibtex</summary>

      ```bibtex
      @inproceedings{gromov2019proximity,
        author = {Gromov, Boris and Abbate, Gabriele and Gambardella, Luca and Giusti, Alessandro},
        title = {Proximity Human-Robot Interaction Using Pointing Gestures and a Wrist-mounted {IMU}},
        booktitle = {2019 IEEE International Conference on Robotics and Automation (ICRA)},
        pages = {8084-8091},
        year = {2019},
        month = may,
        doi = {10.1109/ICRA.2019.8794399},
        issn = {2577-087X},
      }      
      ```
      
    </details>
    
 - B. Gromov, L. Gambardella, and A. Giusti, “Robot Identification and Localization with Pointing Gestures,” in 2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), 2018, pp. 3921–3928.
   [Details](http://people.idsia.ch/~gromov/bibliography/gromov2018robot.html).
    <details>
      <summary>Bibtex</summary>

      ```bibtex
      @inproceedings{gromov2018robot,
        author = {Gromov, Boris and Gambardella, Luca and Giusti, Alessandro},
        title = {Robot Identification and Localization with Pointing Gestures},
        booktitle = {2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
        year = {2018},
        pages = {3921-3928},
        doi = {10.1109/IROS.2018.8594174},
        issn = {2153-0866},
        month = oct,
      }
      ```
  
    </details>

## License

Most of the code is released under the `BSD 3-Clause New` license.

The `volaly_localization` and `volaly_kinematics` are released under `BSD 3-Clause Clear` license that retains the patent rights.
