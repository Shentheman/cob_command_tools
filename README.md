# cob_command_tools
## Version
* This is based on the release version ros-indigo-cob-command-tools:amd64/trusty **0.6.6**-0trusty-20170804-172332-0800, and incorporated with the changes customized for Robbie & Yuri (cob raw3-6) in Interactive Robotics Group.
  * To mix a release version with customizations:
    * `git clone https://github.com/ipa320/cob_command_tools.git`
    * `git checkout 0.6.6`
    * `git remote -v`
    * `git remote rename origin upstream`
    * `git remote add origin <your forked git repo url>`
      * `git remote add origin https://github.com/Shentheman/cob_command_tools.git`
    * `git fetch origin`
    * `git checkout -b <your new branch name>`
      * `git checkout -b irg-raw3-6`
    * `git status`
    * `git add -A`
    * `git commit -am <commit messages>`
    * `git push origin master`
* On 04/01/18, we merge the new changes from the release [0.6.7](https://github.com/ipa320/cob_command_tools/releases)

