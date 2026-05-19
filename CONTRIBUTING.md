# Contributing

## Branch strategy

- `main` — stable, release-ready code. Protected.
- `develop` — integration branch for ongoing development.
- `feature/<short-name>` — feature branches, merged into `develop` via PR.
- `fix/<short-name>` — bug fix branches.

## Workflow

1. Branch from `develop`: `git checkout -b feature/my-feature develop`
2. Commit using [Conventional Commits](https://www.conventionalcommits.org/):
   - `feat:` new feature
   - `fix:` bug fix
   - `docs:` documentation only
   - `refactor:` code refactor without behavior change
   - `test:` test additions
   - `chore:` tooling, CI, config
3. Push and open a PR into `develop`.
4. CI must pass before merge.
5. `develop` → `main` only after a complete project phase.