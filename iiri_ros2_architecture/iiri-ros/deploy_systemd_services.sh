#!/bin/bash

#=============================================================================
# Systemd Service Deployment Script
# Deploys iiri-qr.service and iiri-ros.service
# Author: 唐文浩
# Date: 2025-10-10
# Updated: 2025-10-11 - Added system_bringup package verification
#=============================================================================

# Color definitions
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Service file paths
QR_SERVICE_FILE="$SCRIPT_DIR/iiri-qr.service"
ROS_SERVICE_FILE="$SCRIPT_DIR/iiri-ros.service"

# Systemd directory
SYSTEMD_DIR="/lib/systemd/system"

# Print functions
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_header() {
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}$1${NC}"
    echo -e "${GREEN}========================================${NC}"
}

# Check root privileges
check_root() {
    if [ "$EUID" -ne 0 ]; then
        print_error "This script requires root privileges, please use sudo"
        echo -e "  Usage: ${YELLOW}sudo $0${NC}"
        exit 1
    fi
}

# Check service files exist
check_service_files() {
    print_info "Checking service files..."

    local missing=0

    if [ ! -f "$QR_SERVICE_FILE" ]; then
        print_error "Service file not found: $QR_SERVICE_FILE"
        missing=1
    else
        print_success "Found service file: iiri-qr.service"
    fi

    if [ ! -f "$ROS_SERVICE_FILE" ]; then
        print_error "Service file not found: $ROS_SERVICE_FILE"
        missing=1
    else
        print_success "Found service file: iiri-ros.service"
    fi

    if [ $missing -eq 1 ]; then
        print_error "Some service files are missing, cannot proceed"
        exit 1
    fi
}

# Ensure symbolic link is created/updated
ensure_symlink() {
    print_header "Checking Deployment Symlink"

    local EXPECTED_LINK="/home/wl/autorun/iiri-ros"

    # 如果当前已经是符号链接目标，无需操作
    if [ "$SCRIPT_DIR" == "$EXPECTED_LINK" ]; then
        print_info "Already at expected path: $EXPECTED_LINK"
        return 0
    fi

    # 检查符号链接状态
    if [ -L "$EXPECTED_LINK" ]; then
        local CURRENT_TARGET=$(readlink -f "$EXPECTED_LINK")
        if [ "$CURRENT_TARGET" == "$SCRIPT_DIR" ]; then
            print_success "Symlink already points to this version ✓"
            print_info "  $EXPECTED_LINK -> $SCRIPT_DIR"
            return 0
        else
            print_warning "Symlink points to different version:"
            print_info "  Current: $EXPECTED_LINK -> $CURRENT_TARGET"
            print_info "  New: $EXPECTED_LINK -> $SCRIPT_DIR"
            print_info "Updating symlink to new version..."
        fi
    elif [ -e "$EXPECTED_LINK" ]; then
        print_error "$EXPECTED_LINK exists but is not a symlink!"
        print_error "Please manually handle this directory before deploying."
        print_info "You can:"
        print_info "  1. Backup: mv $EXPECTED_LINK ${EXPECTED_LINK}.backup"
        print_info "  2. Then re-run this script"
        return 1
    else
        print_info "Symlink does not exist, creating new one..."
    fi

    # 创建或更新符号链接
    print_info "Creating/updating symlink..."
    ln -snf "$SCRIPT_DIR" "$EXPECTED_LINK"

    if [ $? -eq 0 ]; then
        print_success "Symlink created: $EXPECTED_LINK -> $SCRIPT_DIR"
        echo ""
        print_info "💡 This allows systemd services to use fixed path /home/wl/autorun/iiri-ros"
        print_info "   while maintaining version-specific deployments for rollback capability."
        echo ""
    else
        print_error "Failed to create symlink"
        return 1
    fi
}

# Check dependencies and workspace
check_dependencies() {
    print_info "Checking dependencies and workspace..."

    local missing=0

    # Check iiri-qr directory
    if [ ! -d "/home/wl/autorun/iiri-qr" ]; then
        print_warning "Directory not found: /home/wl/autorun/iiri-qr"
        print_info "  This will be created when deploying from workspace"
    else
        print_success "Directory exists: /home/wl/autorun/iiri-qr"

        if [ ! -f "/home/wl/autorun/iiri-qr/qr_start.sh" ]; then
            print_warning "Script not found: /home/wl/autorun/iiri-qr/qr_start.sh"
        fi

        if [ ! -f "/home/wl/autorun/iiri-qr/qr_stop.sh" ]; then
            print_warning "Script not found: /home/wl/autorun/iiri-qr/qr_stop.sh"
        fi
    fi

    # Check iiri-ros directory
    if [ ! -d "/home/wl/autorun/iiri-ros" ]; then
        print_warning "Directory not found: /home/wl/autorun/iiri-ros"
        print_info "  This will be created when deploying from workspace"
    else
        print_success "Directory exists: /home/wl/autorun/iiri-ros"

        if [ ! -f "/home/wl/autorun/iiri-ros/start_ros2_iiri_start.sh" ]; then
            print_warning "Script not found: /home/wl/autorun/iiri-ros/start_ros2_iiri_start.sh"
        else
            # Check if the script uses system_bringup (new) or bringup (old)
            if grep -q "ros2 launch system_bringup" "/home/wl/autorun/iiri-ros/start_ros2_iiri_start.sh"; then
                print_success "Startup script uses new system_bringup package ✓"
            elif grep -q "ros2 launch bringup" "/home/wl/autorun/iiri-ros/start_ros2_iiri_start.sh"; then
                print_warning "Startup script uses OLD bringup package - should be updated to system_bringup"
            fi
        fi

        if [ ! -f "/home/wl/autorun/iiri-ros/stop_ros2_iiri_advanced.sh" ]; then
            print_warning "Script not found: /home/wl/autorun/iiri-ros/stop_ros2_iiri_advanced.sh"
        fi

        if [ ! -f "/home/wl/autorun/iiri-ros/install/setup.bash" ]; then
            print_warning "Setup file not found: /home/wl/autorun/iiri-ros/install/setup.bash"
        else
            # Verify system_bringup package is available
            print_info "Verifying system_bringup package availability..."

            # Source the workspace and check for system_bringup
            if bash -c "source /home/wl/autorun/iiri-ros/install/setup.bash && ros2 pkg list | grep -q system_bringup" 2>/dev/null; then
                print_success "system_bringup package is available in workspace ✓"
            else
                print_warning "system_bringup package NOT found in workspace"
                print_info "  Please ensure the workspace includes core_layer with system_bringup"
            fi
        fi
    fi
}

# Copy service files to systemd directory
copy_service_files() {
    print_info "Copying service files to $SYSTEMD_DIR ..."

    # Copy iiri-qr.service
    cp "$QR_SERVICE_FILE" "$SYSTEMD_DIR/"
    if [ $? -eq 0 ]; then
        print_success "Copied successfully: iiri-qr.service"
    else
        print_error "Failed to copy: iiri-qr.service"
        exit 1
    fi

    # Copy iiri-ros.service
    cp "$ROS_SERVICE_FILE" "$SYSTEMD_DIR/"
    if [ $? -eq 0 ]; then
        print_success "Copied successfully: iiri-ros.service"
    else
        print_error "Failed to copy: iiri-ros.service"
        exit 1
    fi

    # Set correct permissions
    chmod 644 "$SYSTEMD_DIR/iiri-qr.service"
    chmod 644 "$SYSTEMD_DIR/iiri-ros.service"
    print_success "Set file permissions"
}

# Install switch-version.sh tool to /home/wl/autorun
install_switch_version_tool() {
    print_info "Installing version switching tool..."

    local SWITCH_VERSION_SRC="$SCRIPT_DIR/switch-version.sh"
    local SWITCH_VERSION_DEST="/home/wl/autorun/switch-version.sh"

    # 检查源文件是否存在
    if [ ! -f "$SWITCH_VERSION_SRC" ]; then
        print_warning "switch-version.sh not found in deployment package"
        print_info "  This is expected for older deployment packages"
        return 0
    fi

    # 复制到 /home/wl/autorun
    cp "$SWITCH_VERSION_SRC" "$SWITCH_VERSION_DEST"
    if [ $? -eq 0 ]; then
        chmod +x "$SWITCH_VERSION_DEST"
        print_success "Installed: $SWITCH_VERSION_DEST"
        echo ""
        print_info "💡 Version management commands:"
        print_info "   List versions:  sudo $SWITCH_VERSION_DEST list"
        print_info "   Switch version: sudo $SWITCH_VERSION_DEST <version>"
        print_info "   Rollback:       sudo $SWITCH_VERSION_DEST rollback"
    else
        print_warning "Failed to install switch-version.sh"
        print_info "  You can manually copy it later if needed"
    fi
}

# Reload systemd daemon
reload_systemd() {
    print_info "Reloading systemd daemon..."
    systemctl daemon-reload
    if [ $? -eq 0 ]; then
        print_success "Systemd daemon reloaded successfully"
    else
        print_error "Failed to reload systemd daemon"
        exit 1
    fi
}

# Enable services
enable_services() {
    print_info "Enabling services for auto-start..."

    # Enable iiri-qr first (iiri-ros depends on it)
    systemctl enable iiri-qr.service
    if [ $? -eq 0 ]; then
        print_success "Enabled auto-start: iiri-qr.service"
    else
        print_error "Failed to enable: iiri-qr.service"
    fi

    # Enable iiri-ros
    systemctl enable iiri-ros.service
    if [ $? -eq 0 ]; then
        print_success "Enabled auto-start: iiri-ros.service"
    else
        print_error "Failed to enable: iiri-ros.service"
    fi
}

# Show service status
show_status() {
    print_header "Service Status"

    echo -e "${BLUE}iiri-qr.service:${NC}"
    systemctl status iiri-qr.service --no-pager | head -10
    echo ""

    echo -e "${BLUE}iiri-ros.service:${NC}"
    systemctl status iiri-ros.service --no-pager | head -10
    echo ""
}

# Start services
start_services() {
    print_info "Starting services..."

    # Start iiri-qr first
    systemctl start iiri-qr.service
    if [ $? -eq 0 ]; then
        print_success "Started successfully: iiri-qr.service"
    else
        print_error "Failed to start: iiri-qr.service"
    fi

    # Wait a moment
    sleep 1

    # Start iiri-ros
    systemctl start iiri-ros.service
    if [ $? -eq 0 ]; then
        print_success "Started successfully: iiri-ros.service"
    else
        print_error "Failed to start: iiri-ros.service"
    fi
}

# Stop services
stop_services() {
    print_info "Stopping services..."

    # Stop iiri-ros first
    systemctl stop iiri-ros.service
    if [ $? -eq 0 ]; then
        print_success "Stopped successfully: iiri-ros.service"
    else
        print_error "Failed to stop: iiri-ros.service"
    fi

    # Stop iiri-qr
    systemctl stop iiri-qr.service
    if [ $? -eq 0 ]; then
        print_success "Stopped successfully: iiri-qr.service"
    else
        print_error "Failed to stop: iiri-qr.service"
    fi
}

# Show usage information
show_usage() {
    echo -e "${GREEN}Systemd Service Deployment Script${NC}"
    echo ""
    echo "Usage:"
    echo -e "  ${YELLOW}sudo $0 [command]${NC}"
    echo ""
    echo "Commands:"
    echo "  install     - Install and enable services (no start)"
    echo "  start       - Start services"
    echo "  stop        - Stop services"
    echo "  restart     - Restart services"
    echo "  status      - Show service status"
    echo "  deploy      - Full deployment (install + enable + start)"
    echo "  help        - Show this help message"
    echo ""
    echo "Examples:"
    echo -e "  ${YELLOW}sudo $0 deploy${NC}      # Full deployment"
    echo -e "  ${YELLOW}sudo $0 status${NC}      # Check status"
    echo -e "  ${YELLOW}sudo $0 restart${NC}     # Restart services"
    echo ""
    echo "NOTE: This script now uses the new system_bringup package (located in core_layer)"
    echo "      Make sure your workspace includes system_bringup before deploying."
    echo ""
}

# Main function
main() {
    check_root

    case "${1:-}" in
        install)
            print_header "Installing Systemd Services"
            check_service_files
            ensure_symlink
            check_dependencies
            copy_service_files
            install_switch_version_tool
            reload_systemd
            enable_services
            print_success "Services installed and enabled for auto-start"
            print_info "Use 'sudo $0 start' to start the services"
            ;;

        deploy)
            print_header "Deploying Systemd Services"
            check_service_files
            ensure_symlink
            check_dependencies
            copy_service_files
            install_switch_version_tool
            reload_systemd
            enable_services
            echo ""
            start_services
            echo ""
            show_status
            print_success "Deployment complete"
            ;;

        start)
            start_services
            sleep 2
            show_status
            ;;

        stop)
            stop_services
            ;;

        restart)
            print_info "Restarting services..."
            stop_services
            sleep 2
            start_services
            sleep 2
            show_status
            ;;

        status)
            show_status
            ;;

        help|--help|-h)
            show_usage
            ;;

        *)
            print_error "Unknown command: $1"
            echo ""
            show_usage
            exit 1
            ;;
    esac
}

# Execute main function
main "$@"
