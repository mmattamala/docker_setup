#!/bin/bash
# This is a helper script to push images to DockerHub or ORI-internal registry
set -e

# Default target
TARGET=none

# Read arguments
for i in "$@"
do
    case $i in
        -t=*|--target=*)
          TARGET=${i#*=}
          echo "[push.sh]: User-set target type is: '$TARGET'"
          shift
          ;;
    esac
done

update_tags_and_push()
{
    local old_tag=$1
    local new_tag=$2

    # Build pytorch geometric docker
    echo "Making new tag [$new_tag] from old tag [$old_tag]"
    docker tag $old_tag $new_tag
    echo "Pushing [$new_tag] to server..."
    docker push $new_tag
}

# Handle different target cases
source targets/$TARGET.sh

# Push
update_tags_and_push $WVN_DESKTOP_TAG $ORI_WVN_DESKTOP_TAG


# # Handle different target cases
# if [[ "$TARGET" == "jetson" ]]; then
#     update_tags_and_push "$PYG_JETSON_TAG" "$ORI_PYG_JETSON_TAG"
#     update_tags_and_push "$WVN_JETSON_TAG" "$ORI_WVN_JETSON_TAG"

# elif [[ "$TARGET" == "desktop" ]]; then
#     update_tags_and_push $WVN_DESKTOP_TAG $ORI_WVN_DESKTOP_TAG

# else
#     echo "Error: unsupported target [$TARGET]"
# fi
