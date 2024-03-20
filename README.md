I you want to work on your own branch do this: (infos from https://docs.gitlab.com/ee/gitlab-basics/start-using-git.html)

1) On your shell, type the following command to add your username:

	git config --global user.name "YOUR_USERNAME"

2) Then verify that you have the correct username:

	git config --global user.name

3) To set your email address, type the following command:

	git config --global user.email "your_email_address@example.com"

4) To verify that you entered your email correctly, type:

	git config --global user.email


5) Go to the master branch to pull the latest changes from there

	git checkout master


6) Download the latest changes in the project

This is for you to work on an up-to-date copy (it is important to do every time you work on a project), while you setup tracking branches.

	git pull REMOTE NAME-OF-BRANCH -u

(REMOTE: origin) (NAME-OF-BRANCH: could be "master" or an existing branch)


7) Work on a branch that has already been created

	git checkout NAME-OF-BRANCH



HOW TO PUSH TO GIT:

1) Add the files you want to push:
	git add <files>

2) Check if file is modified and correctly added (modified files must be green):
	git status

3) Commit your changeset:
	git commit -m "Your changeset comment"

4) Push information to the branch (branch could be master or develop_name):
	git push origin <your branch to push>

