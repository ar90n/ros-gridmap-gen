# Release Process

This document outlines the process for creating releases of the ROS GridMap Generator.

## Pre-Release Checklist

Before creating a release, ensure the following:

- [ ] All tests are passing (`npm test`)
- [ ] Type checking passes (`npm run type-check`)
- [ ] Application builds successfully (`npm run build`)
- [ ] Security audit is clean (`npm audit`)
- [ ] All new features are documented
- [ ] CHANGELOG.md is updated with changes
- [ ] All changes are committed and pushed to main branch

## Release Types

This project follows [Semantic Versioning](https://semver.org/):

- **Patch Release (0.0.x)**: Bug fixes, minor improvements
- **Minor Release (0.x.0)**: New features, backward compatible changes
- **Major Release (x.0.0)**: Breaking changes, major features

## Release Process

### 1. Update CHANGELOG.md

Before releasing, update the CHANGELOG.md:

1. Move items from `[Unreleased]` section to a new version section
2. Add the release date
3. Create a new empty `[Unreleased]` section for future changes

Example:
```markdown
## [Unreleased]

## [0.2.0] - 2024-01-15

### Added
- New feature X
- Enhancement Y

### Fixed
- Bug Z
```

### 2. Create Release

Use one of the following commands based on the type of release:

```bash
# Patch release (0.1.0 → 0.1.1)
npm run release:patch

# Minor release (0.1.0 → 0.2.0)  
npm run release:minor

# Major release (0.1.0 → 1.0.0)
npm run release:major
```

These commands will:
1. Run all pre-release checks (type-check, tests, build)
2. Update the version in package.json
3. Create a commit with the new version
4. Create a git tag

### 3. Push Release

After the release commit and tag are created locally:

```bash
npm run push-release
```

This will:
1. Push the commits to the remote repository
2. Push the tags to the remote repository

### 4. Automated Release Creation

Once the tag is pushed, GitHub Actions will automatically:

1. Run the full test suite
2. Build the application
3. Extract changelog notes for the version
4. Create a GitHub Release with:
   - Release notes from CHANGELOG.md
   - Built application as a ZIP asset
   - Proper release/pre-release marking

## Manual Steps (if needed)

If the automated process fails, you can create a release manually:

1. Go to GitHub > Releases > "Create a new release"
2. Choose the tag that was created
3. Copy the relevant section from CHANGELOG.md
4. Upload the built application as an asset
5. Mark as pre-release if the version contains a hyphen (e.g., v0.2.0-beta)

## Troubleshooting

### Release script fails at pre-release step

Check which step failed:
```bash
# Run individually to identify the issue
npm run type-check
npm run test  
npm run build
```

### Git tag already exists

If you need to recreate a tag:
```bash
# Delete local tag
git tag -d v0.1.0

# Delete remote tag (if pushed)
git push origin --delete v0.1.0
```

### GitHub Actions release fails

1. Check the Actions tab in the GitHub repository
2. Review the logs for the failed step
3. Common issues:
   - CHANGELOG.md format problems
   - Missing permissions
   - Build failures

## Best Practices

1. **Always test locally** before releasing
2. **Keep CHANGELOG.md updated** with every merge to main
3. **Use descriptive commit messages** following conventional commits
4. **Release frequently** to avoid large, complex releases
5. **Tag pre-releases** for beta testing (e.g., v0.2.0-beta.1)

## Emergency Hotfix Process

For critical bugs that need immediate release:

1. Create a hotfix branch from the latest release tag
2. Make the minimal necessary changes
3. Update CHANGELOG.md with the fix
4. Create a patch release directly from the hotfix branch
5. Merge the hotfix back to main

Example:
```bash
git checkout -b hotfix/critical-bug v0.1.0
# Make fixes
git add .
git commit -m "Fix critical bug"
npm run release:patch
npm run push-release
```