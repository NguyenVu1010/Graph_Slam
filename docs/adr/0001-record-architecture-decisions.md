# ADR-0001: Record architecture decisions

- Status: accepted
- Date: 2026-10-22

## Context

The project mixes estimation math, real-time constraints and two deployment targets.
Choices made in passing get lost or silently reversed.

## Decision

Keep ADRs in `docs/adr/`, numbered, one decision each, using `0000-template.md`.
Dependencies, build-system changes and frame-convention changes always need an ADR.
A change that goes against an accepted ADR starts with a new ADR that supersedes it.

## Consequences

Slightly more writing up front; reviewers and AI assistants can read why before editing.
