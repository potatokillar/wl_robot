#!/bin/bash

#=============================================================================
# IIRI-QR Systemd Service Deployment Script
# Author: 唐文浩
# Date: 2025-10-15
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
SERVICE_FILE="$SCRIPT_DIR/iiri-qr.service"
SERVICE_NAME="iiri-qr.service"

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

# Check required files exist
check_required_files() {
    print_info "Checking required files..."

    local missing=0
    local REQUIRED_FILES=("qr" "qr_start.sh" "qr_stop.sh" "iiri-qr.service" "robotrun.py" "server.json")

    for file in "${REQUIRED_FILES[@]}"; do
        if [ ! -f "$SCRIPT_DIR/$file" ]; then
            print_error "Required file not found: $file"
            missing=1
        else
            print_success "Found: $file"
        fi
    done

    if [ $missing -eq 1 ]; then
        print_error "Some required files are missing, cannot proceed"
        exit 1
    fi
}

# Ensure symbolic link is created/updated
ensure_symlink() {
    print_header "Checking Deployment Symlink"

    local EXPECTED_LINK="/home/wl/autorun/iiri-qr"

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
        print_info "💡 This allows systemd service to use fixed path /home/wl/autorun/iiri-qr"
        print_info "   while maintaining version-specific deployments for rollback capability."
        echo ""
    else
        print_error "Failed to create symlink"
        return 1
    fi
}

# Set file permissions
set_permissions() {
    print_info "Setting file permissions..."

    chmod +x "$SCRIPT_DIR/qr"
    chmod +x "$SCRIPT_DIR/qr_start.sh"
    chmod +x "$SCRIPT_DIR/qr_stop.sh"
    chmod +x "$SCRIPT_DIR/robotrun.py"
    chown -R wl:wl "$SCRIPT_DIR"

    print_success "Permissions set successfully"
}

# Copy service file to systemd directory
copy_service_file() {
    print_info "Copying service file to $SYSTEMD_DIR ..."

    cp "$SERVICE_FILE" "$SYSTEMD_DIR/"
    if [ $? -eq 0 ]; then
        print_success "Copied successfully: $SERVICE_NAME"
    else
        print_error "Failed to copy: $SERVICE_NAME"
        exit 1
    fi

    # Set correct permissions
    chmod 644 "$SYSTEMD_DIR/$SERVICE_NAME"
    print_success "Set file permissions"
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

# Enable service
enable_service() {
    print_info "Enabling service for auto-start..."

    systemctl enable "$SERVICE_NAME"
    if [ $? -eq 0 ]; then
        print_success "Enabled auto-start: $SERVICE_NAME"
    else
        print_error "Failed to enable: $SERVICE_NAME"
    fi
}

# Show service status
show_status() {
    print_header "Service Status"

    echo -e "${BLUE}$SERVICE_NAME:${NC}"
    systemctl status "$SERVICE_NAME" --no-pager -l | head -15
    echo ""

    if systemctl is-active --quiet "$SERVICE_NAME"; then
        print_success "Service is running ✓"
    else
        print_warning "Service is not running"
    fi
}

# Start service
start_service() {
    print_info "Starting service..."

    systemctl start "$SERVICE_NAME"
    if [ $? -eq 0 ]; then
        print_success "Started successfully: $SERVICE_NAME"
    else
        print_error "Failed to start: $SERVICE_NAME"
        print_info "Check logs: sudo journalctl -u $SERVICE_NAME -n 50"
    fi
}

# Stop service
stop_service() {
    print_info "Stopping service..."

    systemctl stop "$SERVICE_NAME"
    if [ $? -eq 0 ]; then
        print_success "Stopped successfully: $SERVICE_NAME"
    else
        print_error "Failed to stop: $SERVICE_NAME"
    fi
}

# Show usage information
show_usage() {
    echo -e "${GREEN}IIRI-QR Systemd Service Deployment Script${NC}"
    echo ""
    echo "Usage:"
    echo -e "  ${YELLOW}sudo $0 [command]${NC}"
    echo ""
    echo "Commands:"
    echo "  install     - Install and enable service (no start)"
    echo "  start       - Start service"
    echo "  stop        - Stop service"
    echo "  restart     - Restart service"
    echo "  status      - Show service status"
    echo "  deploy      - Full deployment (install + enable + start)"
    echo "  help        - Show this help message"
    echo ""
    echo "Examples:"
    echo -e "  ${YELLOW}sudo $0 deploy${NC}      # Full deployment"
    echo -e "  ${YELLOW}sudo $0 status${NC}      # Check status"
    echo -e "  ${YELLOW}sudo $0 restart${NC}     # Restart service"
    echo ""
}

# Main function
main() {
    check_root

    case "${1:-}" in
        install)
            print_header "Installing IIRI-QR Systemd Service"
            check_required_files
            ensure_symlink
            set_permissions
            copy_service_file
            reload_systemd
            enable_service
            print_success "Service installed and enabled for auto-start"
            print_info "Use 'sudo $0 start' to start the service"
            ;;

        deploy)
            print_header "Deploying IIRI-QR Systemd Service"
            check_required_files
            ensure_symlink
            set_permissions
            copy_service_file
            reload_systemd
            enable_service
            echo ""
            start_service
            sleep 3
            echo ""
            show_status
            print_success "Deployment complete"
            ;;

        start)
            start_service
            sleep 2
            show_status
            ;;

        stop)
            stop_service
            ;;

        restart)
            print_info "Restarting service..."
            stop_service
            sleep 2
            start_service
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
