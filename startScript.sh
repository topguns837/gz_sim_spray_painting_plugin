#!/bin/bash
# ─────────────────────────────────────────────────────────────────────────────
# Spray Paint Plugin – Startup Menu
# ─────────────────────────────────────────────────────────────────────────────

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE_NAME="spray_paint_plugin"

# ── Colours ───────────────────────────────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
CYAN='\033[0;36m'; BOLD='\033[1m'; RESET='\033[0m'

print_banner() {
    clear
    echo -e "${CYAN}${BOLD}"
    echo "  ╔══════════════════════════════════════════════════╗"
    echo "  ║          GZ SIM – SPRAY PAINT PLUGIN            ║"
    echo "  ╚══════════════════════════════════════════════════╝"
    echo -e "${RESET}"
}

print_menu() {
    echo -e "  ${BOLD}Select an option:${RESET}"
    echo ""
    echo -e "  ${GREEN}[1]${RESET}  Start Stack       – launch gz sim in Docker via tmux"
    echo -e "  ${YELLOW}[2]${RESET}  Code Build        – colcon build inside Docker container"
    echo -e "  ${CYAN}[3]${RESET}  Docker Build      – build the Docker image"
    echo -e "  ${BOLD}[4]${RESET}  Empty Container   – open a bash shell in the Docker container"
    echo -e "  ${RED}[q]${RESET}  Quit"
    echo ""
}

# ── Option 1: Start stack ─────────────────────────────────────────────────────
start_stack() {
    local container="spray_paint_stack"
    echo -e "\n${GREEN}${BOLD}▶ Starting stack...${RESET}\n"

    # Start container detached (all Docker mechanics in run_docker.sh)
    bash "$SCRIPT_DIR/run_scripts/docker/run_docker.sh" \
        "container_name=$container" detach

    # Run tmux setup + attach inside the running container
    docker exec -it "$container" python3 /ws/run_scripts/run_stack.py

    # Clean up when the tmux session ends
    docker stop "$container" 2>/dev/null || true
    docker rm   "$container" 2>/dev/null || true
}

# ── Option 2: Code build (delegates to run_scripts/build_code.py) ─────────────
code_build() {
    echo -e "\n${YELLOW}${BOLD}▶ Building code...${RESET}\n"
    python3 "$SCRIPT_DIR/run_scripts/build_code.py"
}

# ── Option 4: Empty container ─────────────────────────────────────────────────
empty_container() {
    echo -e "\n${BOLD}▶ Opening empty container...${RESET}\n"
    bash "$SCRIPT_DIR/run_scripts/docker/run_docker.sh" empty_container
}

# ── Option 3: Docker build ─────────────────────────────────────────────────────
docker_build() {
    echo -e "\n${CYAN}${BOLD}▶ Building Docker image '${IMAGE_NAME}'...${RESET}\n"

    if ! command -v docker &>/dev/null; then
        echo -e "  ${RED}Error:${RESET} Docker is not installed or not in PATH."
        return 1
    fi

    cd "$SCRIPT_DIR"
    docker build -t "$IMAGE_NAME" .
    local exit_code=$?

    if [ $exit_code -eq 0 ]; then
        echo -e "\n${GREEN}${BOLD}✔ Docker image '${IMAGE_NAME}' built successfully.${RESET}\n"
    else
        echo -e "\n${RED}${BOLD}✘ Docker build failed (exit code ${exit_code}).${RESET}\n"
    fi
    cd "$SCRIPT_DIR"
}

# ── Main loop ─────────────────────────────────────────────────────────────────
while true; do
    print_banner
    print_menu
    read -rp "  Choice: " choice
    echo ""

    case "$choice" in
        1) start_stack      ;;
        2) code_build       ;;
        3) docker_build     ;;
        4) empty_container  ;;
        q|Q)
            echo -e "  ${BOLD}Goodbye.${RESET}\n"
            exit 0
            ;;
        *)
            echo -e "  ${RED}Invalid option.${RESET} Please enter 1, 2, 3, 4 or q."
            ;;
    esac

    echo ""
    read -rp "  Press Enter to return to menu..."
done
