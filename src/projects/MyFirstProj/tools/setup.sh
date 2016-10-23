# Source file in order to make Snapdragon programs from this directory

export SCRIPT_DIR=$(dirname $(readlink -e ${BASH_SOURCE}))

# Pull most recent PX4 Firmware and rebase fork with updated Firmware
# FIXME
#   If getting error: `No submodule mapping in .gitmodules ...`
#   May need to run `git rm --cached <project without submodule>`
# (cd ${PX4_FIRMWARE} && git fetch upstream && git checkout master && git rebase upstream/master && git push -f origin master)

#######################################
# In the event that you are really stuck and would like to reset to
# the upstream/master branch:

#   BE SURE TO BACK UP THE FILES THAT YOU WOULD LIKE TO KEEP!!!
#   This may be done by creating a new branch and pushing that branch
#   After your files are backed up, run:
#   `git fetch upstream`
#   `git checkout master`
#   `git reset --hard upstream/master`
#   `git push origin master --force`

# See this for more details:
# https://stackoverflow.com/questions/9646167/clean-up-a-fork-and-restart-it-from-the-upstream
#######################################

# echo ${PX4_PATH}
