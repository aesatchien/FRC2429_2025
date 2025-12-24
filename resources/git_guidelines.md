## Team 2429: Git & Branching Rules

![Build Status](https://github.com/aesatchien/FRC2429_2025/actions/workflows/build.yml/badge.svg)

### 1. The Golden Rule

- **main is sacred.** It must always be deployable to the robot.
- **No direct pushes.** All code enters `main` via a Pull Request (PR)
  - (unless Cory explicitly says it's OK)
- **Always** `git pull` **before you start any coding**
- The build badge above is our quick sanity check (Python lets a lot through)
- Quick visual tutorial - go have some fun at https://learngitbranching.js.org/ - 
  - do Main and Remote at least through the intro of each 
  - (may have to disable adblock so you can type in the console)

---



| Step | Simple — On Your Own (main only) | Advanced — On a Team (feature branch) | Explanation                                                                                                                                                                                                                            |
|------|----------------------------------|----------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| Clone repository | `git clone <repo-url>` | `git clone <repo-url>` | Creates a local working copy of the remote repository, e.g. `https://github.com/aesatchien/FRC2429_2025.git`.                                                                                                                          |
| Enter repository | `cd <repo>` | `cd <repo>` | Moves into the repository directory.                                                                                                                                                                                                   |
| Check current branch | `git status` | `git status` | Verifies current branch and working tree state. **Repeat as you update files.**                                                                                                                                                        |
| Create feature branch | *(skip — stay on main)* | `git checkout -b feat/<your-name>/<short-description>`<br>e.g. `git checkout -b feat/cory/add-intake` | Feature branches isolate work. Team members must create their own branch using their name and a short, descriptive feature name. **Never reuse someone else’s branch name.** Bad examples: `feat/test`, `feat/newstuff`, `feat/cory2`. |
| Make code changes | Edit files | Edit files | Perform actual development work.                                                                                                                                                                                                       |
| Run tests | `python -m robotpy sim` | `python -m robotpy sim` | Tests should pass before committing. Run this in your robot directory - it must be able to see a robot.py.                                                                                                                             |
| Review changes | `git diff`  | `git diff`  | *(optional)* Shows uncommitted changes before committing. **Repeat as you update files.**                                                                                                                                              |
| Commit changes | `git commit -am "message"` | `git commit -am "message"` | Records changes locally. `-a` is optional if files are already tracked.                                                                                                                                                                |
| Push changes | `git push` *(pushes to upstream if set)* | `git push -u origin feat/cory/add-intake` | Sends commits to remote. `origin` and `-u` are optional but recommended for first push.                                                                                                                                                |
| Sync main branch | `git pull` | `git checkout main && git pull` | Ensures local `main` is up to date before integrating work.                                                                                                                                                                            |
| Update feature branch from main | *(skip)* | `git checkout feat/cory/add-intake && git merge main` | Brings latest `main` changes into the feature branch. This is where conflicts may appear.                                                                                                                                              |
| Resolve conflicts (if any) | *(skip)* | Fix files → `git add .` → `git commit -m "Fix merge conflicts"` → `git push` | Resolve conflicts locally and push updated branch so the PR reflects the fixes.                                                                                                                                                        |
| Open or update Pull Request | *(skip)* | Open or update PR on GitHub | All code enters `main` via PR per the Golden Rule. Describe changes and how they were tested.                                                                                                                                          |
| Merge PR into main (preferred) | *(already on main)* | Merge via GitHub UI | Preferred merge path: preserves review history and enforces no-direct-push policy.                                                                                                                                                     |
| Merge feature into main (CLI exception) | *(already on main)* | `git checkout main && git merge feat/cory/add-intake && git push` | CLI merge is allowed only when explicitly approved.                                                                                                                                                                                    |
| Cleanup after merge | *(not applicable)* | `git branch -d feat/cory/add-intake` | Removes local feature branch after successful merge.                                                                                                                                                                                   |

