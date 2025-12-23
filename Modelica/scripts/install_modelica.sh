#!/bin/bash
# Install OpenModelica on Ubuntu 24.04 / Linux Mint 22
# This script installs OpenModelica and its dependencies

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to print colored messages
print_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if running on Ubuntu or Mint
if [ ! -f /etc/os-release ]; then
    print_error "Cannot determine OS version. This script is for Ubuntu 24.04 / Linux Mint 22 only."
    exit 1
fi

. /etc/os-release

if [[ "$ID" != "ubuntu" && "$ID" != "linuxmint" ]]; then
    print_error "This script is designed for Ubuntu 24.04 or Linux Mint 22 only."
    exit 1
fi

print_info "Detected OS: $PRETTY_NAME"

# Check for required commands
for cmd in curl gpg apt-get; do
    if ! command -v $cmd &> /dev/null; then
        print_error "$cmd is not installed. Please install it first."
        exit 1
    fi
done

print_info "Updating package lists..."
sudo apt-get update

print_info "Installing prerequisites..."
sudo apt-get install -y ca-certificates curl gnupg lsb-release

# Add OpenModelica GPG key
print_info "Adding OpenModelica GPG key..."
if [ ! -f /usr/share/keyrings/openmodelica-keyring.gpg ]; then
    sudo curl -fsSL http://build.openmodelica.org/apt/openmodelica.asc | \
        sudo gpg --dearmor -o /usr/share/keyrings/openmodelica-keyring.gpg
    print_info "GPG key added successfully"
else
    print_warn "GPG key already exists, skipping..."
fi

# Determine architecture
ARCH=$(dpkg --print-architecture)

# Determine codename
if [ "$ID" == "linuxmint" ]; then
    # Linux Mint 22 is based on Ubuntu 24.04 (Noble)
    CODENAME="noble"
else
    CODENAME=$(lsb_release -cs)
fi

print_info "Detected architecture: $ARCH"
print_info "Detected codename: $CODENAME"

# Add OpenModelica repository
REPO_FILE="/etc/apt/sources.list.d/openmodelica.list"
if [ ! -f "$REPO_FILE" ]; then
    print_info "Adding OpenModelica repository..."
    echo "deb [arch=$ARCH signed-by=/usr/share/keyrings/openmodelica-keyring.gpg] \
  https://build.openmodelica.org/apt \
  $CODENAME \
  stable" | sudo tee "$REPO_FILE" > /dev/null
    print_info "Repository added successfully"
else
    print_warn "Repository file already exists, skipping..."
fi

# Update package lists again
print_info "Updating package lists with OpenModelica repository..."
sudo apt-get update

# Install OpenModelica
print_info "Installing OpenModelica..."
sudo apt-get install -y openmodelica

# Verify installation
print_info "Verifying installation..."
if command -v omc &> /dev/null; then
    OMC_VERSION=$(omc --version 2>&1 | head -n 1)
    print_info "OpenModelica installed successfully!"
    print_info "Version: $OMC_VERSION"
    
    # Test compilation
    print_info "Testing OpenModelica compiler..."
    if omc --help &> /dev/null; then
        print_info "OpenModelica compiler is working correctly"
    else
        print_warn "OpenModelica compiler may have issues"
    fi
else
    print_error "OpenModelica installation verification failed"
    exit 1
fi

# Install Python integration tools (optional but recommended)
print_info "Checking for Python integration tools..."
if command -v python3 &> /dev/null; then
    print_info "Python 3 is available. You may want to install OMPython for Python integration:"
    print_info "  pip install OMPython"
else
    print_warn "Python 3 not found. OMPython integration will not be available."
fi

print_info "Installation complete!"
print_info "You can now use OpenModelica with the 'omc' command"
print_info "For Python integration, install OMPython: pip install OMPython"

