# Final Project Setup
This document provides instructions on how to set up the final project for the
aerial robotics course. Please read the following document very carefully. Use
your best judgement when following this document. Do not just copy/paste ---
some commands may have errors in them. It's hard to ensure that the commands
work on every machine identically when I can only test on my own local machine.

## Create a gitlab account
Each team member must create a gitlab account.


## Create a remote gitlab repository
Each team should designate one team member to host their team's code. This step
must only be followed by that one team member.

One team member must create a GameEngine repository on gitlab. Name both the
repository and the slug as 'GameEngine'. Make the repository private --- this
ensures that only you may view this code. Don't want your opponents to see your
secrets! i

In the repository settings/members, add your team members as maintainers to your
repository. This allows them to view and edit the code. Also add Nick and Tucker
as Reporter to the code. We need the ability to audit your code. Our handles
are: @tuckerhaydon @nickMont

## Clone the GameEngine
First, clone a fresh copy of the MediationLayer repository and call it the
'GameEngine'.
```bash
cd ~/Workspace/
git clone https://github.com/tuckerhaydon/MediationLayerStudent.git GameEngine
```

## Set up the new GameEngine repository
Configure your GameEngine repository with your name and email. By configuring
it, all commits that you make will be signed with this information.
```bash
cd ~/Workspace/GameEngine
git config user.name "YOUR NAME HERE"
git config user.email "YOUR EMAIL HERE"
```

There will be three GameEngine repositories that are important to you: your
local copy, the copy on gitlab, and the original MediationLayerStudent
repository. The three of these interact as follows: Tucker will push patches and
changes to the original MediationLayerStudent repository. Your local copy will
have to pull in and merge these changes. Then you will have to push changes in
your local copy to gitlab so they are safe and accessible to your teammates.
These three repositories will be referred to as:
- Local Copy -> local
- Gitlab copy -> origin
- Original MediationLayerStudent -> source

Configure these in the git settings. Note that the lines begining with '#' below
are comments
```
cd ~/Workspace/GameEngine
git remote rename origin source
git remote add origin https://gitlab.com/YOUR_GITLAB_USERNAME/GameEngine.git
# For example:
#   git remote add origin https://gitlab.com/tuckerhaydon/GameEngine.git
```

Now that your repository is configured, push your local copy to gitlab. You will
push to 'origin' if you want to push to gitlab.
```bash
cd ~/Workspace/GameEngine
git push -u origin --all
git push -u origin --tags
```
After pushing, you should see the code on gitlab.

## Pushing changes
You may want to push changes from your local repository to gitlab so that your
teammates may pull those changes.

First, add any new files you created.
```bash
git add LIST_OF_NEW_FILES
```

Next, commit these files and add a message.
```bash
git commit -am "THIS IS WHERE YOUR COMMIT MESSAGE GOES. IT SHOULD DESCRIBE WHAT
YOU CHANGED"
```

Finally, push to origin.
```bash
git push -u origin master
```

## Pulling changes
Any time a change is made to a remote repository (your teammate has pushed to
gitlab or Tucker has changed the original MediationLayerStudent repository), you
will want to pull in and merge these changes. The steps below assume that you
have already committed any changes in your local repository.

If the remote source is changed, pull from source.
```bash
git pull source master
```

If the remote origin is changed, pull from origin.
```bash
git pull origin master
```



