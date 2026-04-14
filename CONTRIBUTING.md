# Contributing

## Building

```bash
sudo apt-get install libproj-dev libtiff-dev libssl-dev
cmake -S . -B build && cmake --build build -j$(nproc)
```

## Pre-commit hooks

This repo uses [pre-commit](https://pre-commit.com) for formatting and linting. Install once per clone:

```bash
pip install pre-commit
pre-commit install
pre-commit install --hook-type commit-msg  # enforces conventional commit messages
```

Hooks run automatically on `git commit`. To run manually:

```bash
pre-commit run --all-files
```

## Commit messages

We use [Conventional Commits](https://www.conventionalcommits.org/) so [release-please](https://github.com/googleapis/release-please) can auto-generate the changelog and version bumps:

- `feat:` — new feature (bumps minor version)
- `fix:` — bug fix (bumps patch version)
- `chore:`, `docs:`, `refactor:`, `test:` — no version bump
- `feat!:` or `BREAKING CHANGE:` in body — bumps major version

## Releasing

Releases are automated by release-please:

1. Push conventional commits to `main`
2. release-please opens/updates a release PR with the version bump and changelog
3. Merge the release PR — a tag and GitHub release are created automatically, and the loader binary is built and attached

## Filing issues

[Open an issue](https://github.com/breuerpeter/px4-rerun-sdk/issues) with a clear description and, if reporting a bug, steps to reproduce.
