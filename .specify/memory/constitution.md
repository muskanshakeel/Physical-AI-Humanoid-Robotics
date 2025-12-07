<!-- Sync Impact Report:
Version change: 0.0.0 -> 1.0.0
Modified principles:
- Core Principles (added)
- Key Standards (added)
- Constraints (added)
- Governance (added)
Added sections:
- Core Principles
- Key Standards
- Constraints
- Governance
Removed sections:
- None
Templates requiring updates:
- .specify/templates/plan-template.md: ✅ updated
- .specify/templates/spec-template.md: ✅ updated
- .specify/templates/tasks-template.md: ✅ updated
- .specify/templates/commands/*.md: ✅ updated
- README.md: ⚠ pending
Follow-up TODOs:
- TODO(README.md): Update references to principles changed.
-->
# Spec-Driven Development Project Constitution
<!-- Example: Spec Constitution, TaskFlow Constitution, etc. -->

## Core Principles

### Spec-first workflow
<!-- Example: I. Library-First -->
No content, code, files, chapters, or configuration changes are created without an approved .spec.md file.
<!-- Example: Every feature starts as a standalone library; Libraries must be self-contained, independently testable, documented; Clear purpose required - no organizational-only libraries -->

### Reproducibility
<!-- Example: II. CLI Interface -->
A fresh clone → npm ci → npm run build must produce an identical output site.
<!-- Example: Every library exposes functionality via CLI; Text in/out protocol: stdin/args → stdout, errors → stderr; Support JSON + human-readable formats -->

### Zero manual edits
<!-- Example: III. Test-First (NON-NEGOTIABLE) -->
No direct editing of generated content or Docusaurus site files unless dictated by an approved spec.
<!-- Example: TDD mandatory: Tests written → User approved → Tests fail → Then implement; Red-Green-Refactor cycle strictly enforced -->

### Traceability
<!-- Example: IV. Integration Testing -->
Every file must reference the spec that created it.
<!-- Example: Focus areas requiring integration tests: New library contract tests, Contract changes, Inter-service communication, Shared schemas -->

### Quality + consistency
<!-- Example: V. Observability, VI. Versioning & Breaking Changes, VII. Simplicity -->
Writing must be clear, technically accurate, and consistent across chapters.
<!-- Example: Text I/O ensures debuggability; Structured logging required; Or: MAJOR.MINOR.BUILD format; Or: Start simple, YAGNI principles -->

### AI-augmented creation


All content generated using Spec-Kit Plus (for spec structure + enforcement) and Claude Code (for implementation).


## Key Standards
<!-- Example: Additional Constraints, Security Requirements, Performance Standards, etc. -->

- Framework: Docusaurus v3 (Classic preset).
- Content location: All book content in /docs as MDX.
- Specs location: All specs stored under /specs, one spec per unit of work (chapter, feature, config).
- Deployment: Automated deployment to GitHub Pages through GitHub Actions.
- Design: Use default theme; minimal or no custom CSS unless approved by spec.
- Licensing:
  - Text: CC-BY-SA 4.0
  - Code: MIT
- Code blocks: Must be runnable and, where possible, testable.
- Naming + structure: All files follow deterministic naming derived from specs.

## Constraints
<!-- Example: Development Workflow, Review Process, Quality Gates, etc. -->

- No content without spec approval (hard rule enforced by Spec-Kit Plus).
- Sidebar depth: Maximum nesting of 3 levels.
- Build time: Must remain under 2 minutes on GitHub Actions.
- Mobile-responsive: Must remain fully responsive using the default Docusaurus theme.
- No manual editing of generated folders (e.g., .docusaurus, build output, etc.).
- Chapters: Minimum 5 fully completed chapters, each created from its own spec.

## Governance
<!-- Example: Constitution supersedes all other practices; Amendments require documentation, approval, migration plan -->

- Constitution supersedes all other practices.
- Amendments require documentation, approval, and a migration plan.
- All PRs/reviews must verify compliance.
- Complexity must be justified.
- Use `[GUIDANCE_FILE]` for runtime development guidance.

### AI Guidelines
- AI tools (e.g., Claude Code) must never create content without referencing a specific approved spec.
- AI must refuse requests that violate this constitution.
- When uncertain or ambiguous → AI must propose a new spec instead of guessing.
- All generated content must embed a comment linking back to its creating spec.

### Success Criteria
- The site is live at: https://<username>.github.io/<repo>/
- `npm ci && npm run build` runs with zero errors on a fresh clone.
- Lighthouse scores: Performance ≥ 90, Accessibility ≥ 95
- All chapters trace directly back to corresponding .spec.md files.
- All code blocks execute correctly when applicable.
- All deployment actions succeed on the main branch.
- The entire workflow remains reproducible with no untracked changes.
- All writing passes clarity checks (Flesch-Kincaid grade ~10–12 recommended for readability).

**Version**: 1.0.0 | **Ratified**: 2025-12-05 | **Last Amended**: 2025-12-05
<!-- Example: Version: 2.1.1 | Ratified: 2025-06-13 | Last Amended: 2025-07-16 -->