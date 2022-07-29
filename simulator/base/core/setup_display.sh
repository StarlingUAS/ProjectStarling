set -e 
echo "--------------- Start setup_display.sh ----------------------"

if [ "$ENABLE_VIRTUAL_FRAMEBUFFER" = true ]; then
    echo "Creating virtual display using virtual framebuffer"
    Xvfb -displayfd 3 -screen 0 1600x900x24 3>/tmp/displaynum &
    while [ "$(cat /tmp/displaynum)" == "" ]; do sleep 0.1; done
    export DISPLAY=:$(cat /tmp/displaynum)

    echo "Waiting for display to become available"
    while [ ! -e /tmp/.X11-unix/X${DISPLAY#:} ]; do echo "Wait for X server.."; sleep 0.1; done
else
    echo "Virtual Framebuffer Disabled"
fi
echo "--------------- End setup_display.sh ----------------------"
