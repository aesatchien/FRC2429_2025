## Team 2429: Git & Branching Rules

![Build Status](https://github.com/aesatchien/FRC2429_2025/actions/workflows/build.yml/badge.svg)

### 1. The Golden Rule

- **main is sacred.** It must always be deployable to the robot.
- **No direct pushes.** All code enters `main` via a Pull Request (PR)
  - (unless Cory explicitly says it's OK)
- The build badge above is our quick sanity check (Python lets a lot through)
- Go have some fun at https://learngitbranching.js.org/ - do Main and Remote at least through the intro of each

---

| Step | Simple — On Your Own (main only) | Advanced — On a Team (feature branch) | Explanation |
|------|----------------------------------|----------------------------------------|-------------|
| Clone repository | `git clone <repo-url>` | `git clone <repo-url>` | Creates a local working copy of the remote repository, e.g. `https://github.com/aesatchien/FRC2429_2025.git`. |
| Enter repository | `cd <repo>` | `cd <repo>` | Moves into the repository directory. |
| Check current branch | `git status` | `git status` | Verifies current branch and working tree state. |
| Create feature branch | *(skip — stay on main)* | `git checkout -b feat/cory/add-intake` | Feature branches isolate work; skipped in solo workflow. |
| Make code changes | Edit files | Edit files | Perform actual development work. |
| Run tests | `python -m robotpy sim` | `python -m robotpy sim` | Tests should pass before committing. This is the standard Team 2429 local simulation command. |
| Review changes | `git diff` *(optional)* | `git diff` *(optional)* | Shows uncommitted changes before committing. |
| Commit changes | `git commit -am "message"` | `git commit -am "message"` | Records changes locally. `-a` is optional if files are already tracked. |
| Push changes | `git push` *(pushes to upstream if set)* | `git push -u origin feat/cory/add-intake` | Sends commits to remote. `origin` and `-u` are optional but recommended for first push. |
| Open Pull Request | *(skip)* | Open PR on GitHub | All code enters `main` via PR per the Golden Rule. Describe changes and testing. |
| Address review feedback | *(skip)* | Make requested changes, then `git commit` and `git push` | Reviews may require fixes or clarification before merge. |
| Resolve merge conflicts (if any) | *(skip)* | `git checkout feat/cory/add-intake` → `git merge main` | If GitHub reports conflicts, resolve them locally and commit the fix. |
| Update main branch | `git pull` | `git checkout main && git pull` | Syncs local `main` with remote. Explicit branch is optional if already on `main`. |
| Merge work (Option A: CLI) | *(already on main)* | `git merge feat/cory/add-intake` | Integrates feature work into `main` from the CLI. Allowed only when explicitly approved. |
| Merge work (Option B: Web PR) | *(already on main)* | Merge via GitHub UI | Integrates feature work into `main` via the PR merge button. Preferred for review visibility and enforcing the no-direct-push rule. |
| Delete feature branch | *(not applicable)* | `git branch -d feat/cory/add-intake` | Cleans up local branch after merge. |
