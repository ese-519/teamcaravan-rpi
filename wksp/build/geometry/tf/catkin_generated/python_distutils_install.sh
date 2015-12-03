#!/bin/sh -x

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

cd "/home/ubuntu/hemo_code/new_code/wksp/src/geometry/tf"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
/usr/bin/env \
    PYTHONPATH="/home/ubuntu/hemo_code/new_code/wksp/install/lib/python2.7/dist-packages:/home/ubuntu/hemo_code/new_code/wksp/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/ubuntu/hemo_code/new_code/wksp/build" \
    "/usr/bin/python" \
    "/home/ubuntu/hemo_code/new_code/wksp/src/geometry/tf/setup.py" \
    build --build-base "/home/ubuntu/hemo_code/new_code/wksp/build/geometry/tf" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/ubuntu/hemo_code/new_code/wksp/install" --install-scripts="/home/ubuntu/hemo_code/new_code/wksp/install/bin"
