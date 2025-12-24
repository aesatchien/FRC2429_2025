# Team 2429: Git & Branching Rules
![Build Status](https://github.com/aesatchien/FRC2429_2025/actions/workflows/build.yml/badge.svg)
## 1. The Golden Rule
* main is sacred. It must always be deployable to the robot.
* No Direct Pushes. All code enters main via a Pull Request (PR). 
* (unless Cory says it's ok)
* that badge above tells us if our code is kinda safe (python allows a lot)

---

## 2. The Standard Workflow

Step 1: Sync your Local Environment

`git checkout main`

`git pull origin main`

Step 2: Create a Feature Branch

`git checkout -b feat/your-name/your-feature-name`

for example, 

`git checkout -b feat/cory/tankbot-shooter`

Step 3: Make Your Code Changes and Local Test

`python -m robotpy sim`

Step 4: Commit your Work

`git add .`

`git commit -m "Description of change"`

Step 5: Push to GitHub

`git push origin feat/your-name/your-feature-name`

Step 6: The Pull Request (PR)
1. Go to GitHub and click "Compare & pull request".
2. Describe what was changed and how it was tested.
3. Assign a Lead or Mentor as a Reviewer.

Step 7: Cleanup After Merge

`git checkout main`

`git pull origin main`

`git branch -d feat/your-feature-name`

---

## 3. Merge Conflicts
If GitHub says "This branch has conflicts," do not use the web editor. 
1. `git checkout your-branch`
2. `git merge main`
3. Fix the conflicts.
4. `git add .`
5. `git commit -m "Fix merge conflicts"`
6. `git push origin your-branch`
