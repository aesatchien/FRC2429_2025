
#  Team 2429: Git & Branching Rules

### 1. The Golden Rule of `main`
* **`main` is sacred.** The code in `main` should always be able to deploy to the robot and run without crashing.
* **Never** commit or push directly to `main`. All changes must come through a Pull Request (PR).
* ~(unless Cory says it's ok)~

### 2. Branching Protocol
* **One Task, One Branch:** Don't bundle unrelated changes. It makes debugging impossible.
* **Naming Convention:** Use descriptive, lowercase names.
    * **Good:** `feat/gyro-auto-reset`, `fix/drivetrain-deadband`
    * **Bad:** `stuff`, `cj-working`, `fix1`

### 3. The Pull Request (PR) Workflow
1. **Push:** `git push origin your-branch-name`.
2. **Open PR:** Open on GitHub immediately (use **Draft** status if still in progress).
3. **The Proof:** State how you tested it (e.g., "Tested in Sim" or "Ran on practice chassis").
4. **Review:** A Lead or Mentor must approve the code before it is merged.

### 4. Repository Hygiene
* **Stay Updated:** `git pull origin main` before starting any new work.
* **Cleanup:** Periodically remove merged local branches to keep your workspace clear:
  `git branch --merged | grep -v "\*" | grep -v "main" | xargs -n 1 git branch -d`

### 5. Merge Conflicts
* If GitHub reports conflicts, **do not** use the web editor to fix them.
* Resolve conflicts locally with a Lead Programmer to ensure no logic is lost.

![Build Status](https://github.com/aesatchien/FRC2429_2025/actions/workflows/build.yml/badge.svg)