# Security Policy

## Supported Versions

| Version | Supported          |
| ------- | ------------------ |
| 0.1.x   | :white_check_mark: |
| < 0.1   | :x:                |

## Reporting a Vulnerability

**Please do NOT open a public issue for security vulnerabilities.**

If you discover a security vulnerability in AeroForge, please report it privately to:

**Email**: [your-email@example.com]
**Subject**: [SECURITY] Brief description

### What to Include

- **Description** of the vulnerability
- **Steps to reproduce** (with minimal example if possible)
- **Impact assessment** (what can an attacker do?)
- **Affected versions**
- **Suggested fix** (if you have one)

### Response Timeline

- **Acknowledgment**: Within 48 hours
- **Initial assessment**: Within 1 week
- **Fix timeline**: Depends on severity
  - Critical: 1-7 days
  - High: 1-4 weeks
  - Medium: 1-3 months
  - Low: Best effort

### Disclosure Policy

- We will coordinate a public disclosure date with you
- Credit will be given to reporters (unless you wish to remain anonymous)
- CVE IDs will be requested for critical vulnerabilities

## Security Best Practices for Users

### Drone Safety

1. **Never fly near people or property** without explicit consent
2. **Always maintain visual line of sight** with a human pilot at RC controls
3. **Test in simulator first** before connecting to real hardware
4. **Use geofencing** and speed limits in production configs
5. **Comply with local regulations** (FAA Part 107, etc.)

### Software Security

1. **Keep dependencies updated**: `git submodule update --remote vcpkg`
2. **Review configs before flight**: Ensure safety limits are set correctly
3. **Validate inputs**: Don't run configs from untrusted sources
4. **Log telemetry**: Keep `runs/` logs for post-incident analysis
5. **Network security**: If using RTSP/network streams, use encrypted connections

### Known Limitations

- **DJI SDK integration**: Experimental; use dry-run mode for testing
- **No authenticated command channel**: Commands are not cryptographically signed
- **No obstacle avoidance**: v0.1 assumes clear line of sight to target

## Bug Bounty Program

Currently, AeroForge does not have a formal bug bounty program. However, significant security findings will be acknowledged in release notes and the contributor will receive credit.

---

Thank you for helping keep AeroForge and its users safe!
