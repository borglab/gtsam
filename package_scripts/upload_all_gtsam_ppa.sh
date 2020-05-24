#!/bin/bash

function show_help {
    echo "USAGE:"
    echo ""
    echo "- to display this help: "
    echo "upload_all_gtsam_ppa.sh -h or -?"
    echo ""
    echo "- to upload to your PPA: "
    echo "upload_all_gtsam_ppa.sh -p ppa:your_name/name_of_ppa"
    echo ""
}

while getopts "h?p:" opt; do
    case "$opt" in
    h|\?)
        show_help
        exit 0
        ;;
    p)  ppa_name=$OPTARG
        ;;
    esac
done

if [ -z ${ppa_name+x} ]; then
    show_help
    exit -1
fi


find . -name '*.changes' | xargs -I FIL dput ${ppa_name} FIL
