#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
    DESTDIR_ARG="--root=$DESTDIR"
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/jacob/Desktop/URDF-Model/src/arbotix_ros/arbotix_sensors"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/jacob/Desktop/URDF-Model/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/jacob/Desktop/URDF-Model/install/lib/python2.7/dist-packages:/home/jacob/Desktop/URDF-Model/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/jacob/Desktop/URDF-Model/build" \
    "/usr/bin/python2" \
    "/home/jacob/Desktop/URDF-Model/src/arbotix_ros/arbotix_sensors/setup.py" \
    build --build-base "/home/jacob/Desktop/URDF-Model/build/arbotix_ros/arbotix_sensors" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/jacob/Desktop/URDF-Model/install" --install-scripts="/home/jacob/Desktop/URDF-Model/install/bin"
