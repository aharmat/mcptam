# mcptam [![travis][travis_status]][travis_page]

MCPTAM is a set of ROS nodes for running Real-time 3D Visual Simultaneous
Localization and Mapping (SLAM) using Multi-Camera Clusters. It includes tools
for calibrating both the intrinsic and extrinsic parameters of the individual
cameras within the rigid camera rig.

Visit the MCPTAM website (https://github.com/wavelab/mcptam).

For more information, refer to the [MCPTAM Wiki][mcptam_wiki] 

A Getting-Started Guide is available on the Wiki, or a snapshot can be found in
the file Getting-Started.pdf.

If you use this software, please consider citing the following papers:

    A. Harmat, M. Trentini and I. Sharf "Multi-Camera Tracking and Mapping for
    Unmanned Aerial Vehicles in Unstructured Environments" in Journal of
    Intelligent and Robotic Systems, vol. 78, no. 2, pp. 291-317, May 2015
    (http://link.springer.com/article/10.1007%2Fs10846-014-0085-y)
 
    Michael J. Tribou, Adam Harmat, David W.L. Wang, Inna Sharf, and Steven L.
    Waslander. Multi-camera parallel tracking and mapping with non-overlapping
    fields of view. In The International Journal of Robotics Research,
    34(12):1480-1500, October 2015. doi:10.1177/0278364915571429
    (http://ijr.sagepub.com/content/early/2015/04/22/0278364915571429.abstract).


# Licence

    *************************************************************************
     *  
     *  
     *  Copyright 2014  Adam Harmat (McGill University) 
     *                      [adam.harmat@mail.mcgill.ca]
     *                  Michael Tribou (University of Waterloo)
     *                      [mjtribou@uwaterloo.ca]
     *
     *  Multi-Camera Parallel Tracking and Mapping (MCPTAM) is free software:
     *  you can redistribute it and/or modify it under the terms of the GNU 
     *  General Public License as published by the Free Software Foundation,
     *  either version 3 of the License, or (at your option) any later
     *  version.
     *
     *  This program is distributed in the hope that it will be useful,
     *  but WITHOUT ANY WARRANTY; without even the implied warranty of
     *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
     *  GNU General Public License for more details.
     *
     *  You should have received a copy of the GNU General Public License
     *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
     *  
     *  MCPTAM is based on the Parallel Tracking and Mapping (PTAM) software.
     *  Copyright 2008 Isis Innovation Limited
     *  
     *  
     *************************************************************************


[mcptam_wiki]: https://github.com/wavelab/mcptam/wiki
[travis_page]: https://travis-ci.org/wavelab/mcptam
[travis_status]: https://travis-ci.org/wavelab/mcptam.svg