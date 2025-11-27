#!/bin/bash

# Bash completion for build_layered.sh script
# This script provides auto-completion for layer names, parameters, and architectures

_build_layered_completion() {
    local cur prev opts layers params
    COMPREPLY=()
    cur="${COMP_WORDS[COMP_CWORD]}"
    prev="${COMP_WORDS[COMP_CWORD-1]}"
    
    # Define available layers
    layers="core_layer hardware_layer perception_layer intelligence_layer application_layer"
    
    # Define available parameters
    params="-c --ceres help"
    
    # Define available architectures
    architectures="x86 arm"
    
    # Get the script name (handle both ./build_layered.sh and script/build_layered.sh)
    local script_name="${COMP_WORDS[0]##*/}"
    
    # Only provide completion for build_layered.sh
    if [[ "$script_name" != "build_layered.sh" ]]; then
        return 0
    fi
    
    # Check if we're completing after a parameter that doesn't take arguments
    case "$prev" in
        -c|--ceres)
            # After -c or --ceres, suggest layers or other parameters
            COMPREPLY=( $(compgen -W "$layers $params" -- "$cur") )
            return 0
            ;;
    esac
    
    # Check if current word starts with a dash (parameter)
    if [[ "$cur" == -* ]]; then
        COMPREPLY=( $(compgen -W "$params" -- "$cur") )
        return 0
    fi
    
    # Check if we already have a layer specified
    local has_layer=false
    for word in "${COMP_WORDS[@]:1}"; do
        case "$word" in
            core_layer|hardware_layer|perception_layer|intelligence_layer|application_layer)
                has_layer=true
                break
                ;;
        esac
    done
    
    # If no layer specified yet, suggest layers, parameters, and architectures
    if [[ "$has_layer" == false ]]; then
        COMPREPLY=( $(compgen -W "$layers $params $architectures" -- "$cur") )
    else
        # If layer already specified, suggest remaining parameters and architectures
        local remaining_options=""
        
        # Add remaining parameters
        for param in $params; do
            local found=false
            for word in "${COMP_WORDS[@]:1}"; do
                if [[ "$word" == "$param" ]]; then
                    found=true
                    break
                fi
            done
            if [[ "$found" == false ]]; then
                remaining_options="$remaining_options $param"
            fi
        done
        
        # Add remaining architectures
        for arch in $architectures; do
            local found=false
            for word in "${COMP_WORDS[@]:1}"; do
                if [[ "$word" == "$arch" ]]; then
                    found=true
                    break
                fi
            done
            if [[ "$found" == false ]]; then
                remaining_options="$remaining_options $arch"
            fi
        done
        
        COMPREPLY=( $(compgen -W "$remaining_options" -- "$cur") )
    fi
    
    return 0
}

# Register the completion function
complete -F _build_layered_completion build_layered.sh
complete -F _build_layered_completion ./build_layered.sh
complete -F _build_layered_completion script/build_layered.sh
complete -F _build_layered_completion ./script/build_layered.sh