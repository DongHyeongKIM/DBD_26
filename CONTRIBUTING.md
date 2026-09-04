# Contributing

## Workflow

1. Create a branch from `main` using a short name such as `feat/battery-parser` or `fix/csv-loading`.
2. Stage only related changes and review them with `git diff --cached`.
3. Run the available checks. At minimum, run `git diff --cached --check`.
4. Commit using the Conventional Commits format described below.
5. Push the branch and open a pull request.
6. Merge only after the pull request checks pass and at least one reviewer approves it.

Do not commit directly to `main`. Configure the repository's GitHub branch protection rules to require a pull request and one approving review before merging.

## Commit Messages

Use this format:

```text
<type>(optional-scope): <short description>
```

Allowed types:

- `feat`: a user-facing feature
- `fix`: a bug fix
- `docs`: documentation only
- `style`: formatting with no behavior change
- `refactor`: code restructuring with no behavior change
- `perf`: a performance improvement
- `test`: tests only
- `build`: build system or dependency changes
- `ci`: continuous integration changes
- `chore`: maintenance work
- `revert`: revert a previous commit

Examples:

```text
feat(parser): plot filtered battery telemetry
fix(parser): resolve data path from project root
docs: document review workflow
```

Keep the subject in imperative mood, start it with a lowercase letter, omit the final period, and keep it at 72 characters or fewer. Add a body when the reason or tradeoff is not obvious. Use `BREAKING CHANGE:` in the footer for incompatible changes.

## Local Setup

Enable the version-controlled hooks after cloning:

```bash
git config core.hooksPath .githooks
```

The hooks reject malformed commit messages and commits made directly on `main`. Hooks can be bypassed locally, so GitHub branch protection remains the authoritative review gate.

## Review Checklist

- The change has one clear purpose and contains no unrelated files or datasets.
- Paths are portable and no credentials, personal data, or generated artifacts are committed.
- Input assumptions and failure behavior are explicit.
- Numerical calculations, units, coordinate frames, and sign conventions are checked.
- Relevant checks have been run, or the reason they could not be run is documented in the pull request.
