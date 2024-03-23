
AVAILABLE_SECTIONS=( "$(wget http://voxl-packages.modalai.com/dists/qrb5165 -q -O- | grep ' -' | cut -d '>' -f 2 | cut -d '/' -f 1 )" )

AVAILABLE_DISTS=( "$( wget http://voxl-packages.modalai.com/dists -q -O- | grep ' -' | cut -d '>' -f 2 | cut -d '/' -f 1 )" )

__make_package_sh(){
    local OPTS=('help ipk deb timestamp')

    for i in $(seq 1 $((${COMP_CWORD}-1)) ); do
        ## Assume that we're not looking for help if we already have an argument
        OPTS=("${OPTS[@]/help}")
        ## convert argument to lower case for robustness
        arg=$(echo "${COMP_WORDS[i]}" | tr '[:upper:]' '[:lower:]')
        case ${arg} in
            "-t"|"timestamp"|"--timestamp")
                OPTS=("${OPTS[@]/timestamp}")
                ;;
            "-d"|"deb"|"debian"|"--deb"|"--debian")
                OPTS=("${OPTS[@]/deb}")
                ;;
            "-i"|"ipk"|"opkg"|"--ipk"|"--opkg")
                OPTS=("${OPTS[@]/ipk}")
                ;;
            *)
                COMPREPLY=()
                return 0
        esac
    done

    COMPREPLY=( $(compgen -W '${OPTS}' -- ${COMP_WORDS[COMP_CWORD]}) )
    return 0
}

complete -F __make_package_sh ./make_package.sh

__build_sh(){
    local DISTS=( "$( cat build.sh | grep AVAILABLE | cut -d '"' -f 2 )" )
    COMPREPLY=(" ")
    if [ "$COMP_CWORD" -eq 1 ]; then
        COMPREPLY=( $(compgen -W '${DISTS}' -- ${COMP_WORDS[COMP_CWORD]}) )
        return 0
    fi
}

complete -F __build_sh ./build.sh

__deploy_sh(){
    echo -e "\nDeploy script should not be run from within a docker image"
    COMPREPLY=()
    return 0
}

complete -F __deploy_sh ./deploy_to_voxl.sh

__install_build_deps_sh(){
    local DISTS=( "$( cat install_build_deps.sh | grep AVAILABLE | cut -d '"' -f 2 )" )
    COMPREPLY=(" ")
    if [ "$COMP_CWORD" -eq 1 ]; then
        COMPREPLY=( $(compgen -W '${DISTS}' -- ${COMP_WORDS[COMP_CWORD]}) )
        return 0
    elif [ "$COMP_CWORD" -eq 2 ]; then
        COMPREPLY=( $(compgen -W '${AVAILABLE_SECTIONS}' -- ${COMP_WORDS[COMP_CWORD]}) )
        return 0
    fi
}

complete -F __install_build_deps_sh ./install_build_deps.sh
