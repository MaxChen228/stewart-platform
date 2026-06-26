# Codex Project Setup

This directory contains project-scoped Codex settings and optional custom
agents. It is intentionally small:

- `config.toml` only sets project instruction discovery and subagent limits.
- `agents/` contains custom agents for explicit subagent workflows.

Reusable workflows live in `.agents/skills/`, which is the repo skill location
documented by the current Codex manual.
