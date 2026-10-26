---
name: development-rules
description: Apply strict minimal-change development rules for coding tasks, especially Streamlit debugging, PoC work, notebooks, small apps, and repository edits. Use when the user asks Codex to follow development rules, keep changes minimal, avoid unsolicited UI or analysis additions, fix Streamlit startup issues, or work on PoC-stage code where validation matters more than polish.
---

# Development Rules

## Core Rule

Limit changes to the requested scope. Prefer the smallest edit that satisfies the user's request.

## Minimal Change Principle

- Modify only what the user requested.
- Do not add new features unless explicitly requested.
- Do not improve UI unless explicitly requested.
- Do not add analysis functionality unless explicitly requested.
- Do not create demo data unless explicitly requested.
- Do not turn a PoC into a polished deliverable unless explicitly requested.

## Streamlit Tasks

When asked to check or fix Streamlit startup, prioritize in this order:

1. Resolve errors.
2. Confirm the app starts.
3. Perform the smallest display check needed.

Add analysis screens, charts, dashboards, controls, or layout improvements only when the user explicitly asks for them.

## PoC Tasks

For PoC-stage work, prioritize validation over completeness.

- Implement only what is necessary to test the current hypothesis.
- Do not implement beyond MVP scope.
- Do not prebuild features that may be useful later.
