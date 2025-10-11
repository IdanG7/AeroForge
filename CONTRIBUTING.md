# Contributing to AeroForge

Thank you for your interest in contributing to AeroForge! This document provides guidelines for contributing to the project.

## Code of Conduct

This project adheres to a Code of Conduct. By participating, you are expected to uphold this code. Please read [CODE_OF_CONDUCT.md](CODE_OF_CONDUCT.md).

## How Can I Contribute?

### Reporting Bugs

Before creating a bug report, please check existing issues to avoid duplicates. When creating a bug report, include:

- **Clear title and description**
- **Steps to reproduce** the issue
- **Expected vs. actual behavior**
- **Environment**: OS, compiler version, build config
- **Logs**: Include relevant console output or log files from `runs/`

Use the [Bug Report template](.github/ISSUE_TEMPLATE/bug_report.md).

### Suggesting Features

Feature requests are welcome! Please use the [Feature Request template](.github/ISSUE_TEMPLATE/feature_request.md) and include:

- **Use case**: Why is this feature valuable?
- **Proposed API**: What would the interface look like?
- **Alternatives considered**: Other ways to achieve the same goal

### Pull Requests

1. **Fork** the repository and create a branch from `main`
2. **Make your changes** following the coding standards below
3. **Add tests** for new functionality
4. **Update documentation** if you changed APIs or configs
5. **Run tests**: `ctest --preset <platform>-tests`
6. **Format code**: `clang-format -i <files>` (see `.clang-format`)
7. **Submit PR** with a clear description of changes

**PR Checklist:**
- [ ] All CI checks pass (build + tests + lint)
- [ ] Code is formatted (clang-format)
- [ ] New tests added (if applicable)
- [ ] Documentation updated (if applicable)
- [ ] No "TODO" or "FIXME" markers left in final commit
- [ ] Commit messages follow [Conventional Commits](https://www.conventionalcommits.org/)

## Coding Standards

### Style

- **C++ Standard**: C++20
- **Formatting**: `.clang-format` (LLVM-based, 110 col limit, Allman braces)
- **Linting**: `.clang-tidy` (modernize, readability, performance, bugprone)
- **Naming**:
  - Classes/Structs: `PascalCase`
  - Functions/Methods: `snake_case`
  - Variables: `snake_case`
  - Private members: `trailing_underscore_`
  - Constants: `kPascalCase`

### Best Practices

- **RAII**: Manage resources with smart pointers (`std::unique_ptr`, `std::shared_ptr`)
- **Const-correctness**: Mark methods `const` when they don't modify state
- **Avoid raw pointers**: Prefer references or smart pointers
- **Error handling**: Use `std::optional`, `std::expected` (C++23), or exceptions for unrecoverable errors
- **Logging**: Use spdlog with appropriate levels (debug, info, warn, error)
- **Thread safety**: Document thread ownership; prefer SPSC queues over mutexes

### Git Workflow

**Conventional Commits:**
```
feat: add YOLO detector plugin
fix: correct Kalman filter covariance update
docs: update pipeline architecture diagram
test: add integration test for ArUco detector
build: upgrade vcpkg baseline to latest
```

**Branch Naming:**
- `feat/description` for new features
- `fix/description` for bug fixes
- `docs/description` for documentation
- `refactor/description` for code cleanup

## Testing

- **Unit tests**: Test individual components in isolation (e.g., PID, Kalman, ring buffer)
- **Integration tests**: Test end-to-end flows (e.g., detector on synthetic images)
- **Use Catch2**: `TEST_CASE`, `SECTION`, `REQUIRE`, `REQUIRE_APPROX`
- **Coverage**: Aim for >80% line coverage on new code

Example:
```cpp
#include <catch2/catch_test_macros.hpp>
#include "aeroforge/math.hpp"

TEST_CASE("PID clamping", "[control]") {
  af::PIDController pid({1.0, 1.0, 1.0}, {0, 0, 0}, {0, 0, 0}, 10.0);
  Eigen::Vector3d output = pid.compute({100, 100, 100}, 0.1);
  REQUIRE(output.norm() <= 10.0);
}
```

## Documentation

- **Header files**: Document public APIs with Doxygen-style comments
- **README**: Keep quick start up-to-date
- **Architecture docs**: Update `docs/ARCHITECTURE.md` for major changes
- **Config examples**: Add new YAML examples to `configs/` for new modules

## Security

If you discover a security vulnerability, **DO NOT** open a public issue. Email the maintainer directly at [your-email@example.com] with:
- Description of the vulnerability
- Steps to reproduce
- Potential impact

See [SECURITY.md](SECURITY.md) for details.

## Getting Help

- **Discord**: [Join our server](https://discord.gg/YOUR_INVITE)
- **Discussions**: [GitHub Discussions](https://github.com/YOUR_USERNAME/aeroforge/discussions)
- **Issues**: [GitHub Issues](https://github.com/YOUR_USERNAME/aeroforge/issues)

---

Thank you for contributing to AeroForge! 🚀
