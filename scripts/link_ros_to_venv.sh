uv sync

VENV_DIR="${1:-.venv}"
PTH_FILE="ros2.pth"

PYTHON_VERSION=$("$VENV_DIR/bin/python" -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')")
SITE_PACKAGES="$VENV_DIR/lib/python${PYTHON_VERSION}/site-packages"

ROS_PREFIX="/opt/ros/${ROS_DISTRO}"

if [ -d "$ROS_PREFIX/install" ]; then
  ROS_PKG_PREFIX="$ROS_PREFIX/install"
else
  ROS_PKG_PREFIX="$ROS_PREFIX"
fi

cat > "$SITE_PACKAGES/$PTH_FILE" << EOL
/usr/lib/python3/dist-packages
${ROS_PKG_PREFIX}/lib/python${PYTHON_VERSION}/site-packages
${ROS_PKG_PREFIX}/local/lib/python${PYTHON_VERSION}/dist-packages
EOL

source "$VENV_DIR/bin/activate"
