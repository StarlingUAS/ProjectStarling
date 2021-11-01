set -e 
echo "--------------- Start setup_display.sh ----------------------"

if [ "$ENABLE_VIRTUAL_FRAMEBUFFER" = true ]; then
    echo "Creating virtual display using virtual framebuffer"
    export DISPLAY=:1
    rm -f /tmp/.X1-lock
    Xvfb $DISPLAY -screen 0 1600x900x24 &

    echo "Waiting for display to become available"
    while [ ! -e /tmp/.X11-unix/X1 ]; do echo "Wait for X server.."; sleep 0.1; done
else
    echo "Virtual Framebuffer Disabled"
fi
echo "--------------- End setup_display.sh ----------------------"
