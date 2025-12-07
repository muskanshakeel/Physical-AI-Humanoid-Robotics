---
id: 0001
title: Amend constitution to v1.0.0
stage: constitution
date: 2025-12-05
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: user
command: /sp.constitution
labels: ["constitution","amendment"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:

---

## Prompt

Core Principles

Spec-first workflow: No content, code, files, chapters, or configuration changes are created without an approved .spec.md file.

Reproducibility: A fresh clone → npm ci → npm run build must produce an identical output site.

Zero manual edits: No direct editing of generated content or Docusaurus site files unless dictated by an approved spec.

Traceability: Every file must reference the spec that created it.

Quality + consistency: Writing must be clear, technically accurate, and consistent across chapters.

AI-augmented creation: All content generated using Spec-Kit Plus (for spec structure + enforcement) and Claude Code (for implementation).

Key Standards

Framework: Docusaurus v3 (Classic preset).

Content location: All book content in /docs as MDX.

Specs location: All specs stored under /specs, one spec per unit of work (chapter, feature, config).

Deployment: Automated deployment to GitHub Pages through GitHub Actions.

Design: Use default theme; minimal or no custom CSS unless approved by spec.

Licensing:

Text: CC-BY-SA 4.0

Code: MIT

Code blocks: Must be runnable and, where possible, testable.

Naming + structure: All files follow deterministic naming derived from specs.

Constraints

No content without spec approval (hard rule enforced by Spec-Kit Plus).

Sidebar depth: Maximum nesting of 3 levels.

Build time: Must remain under 2 minutes on GitHub Actions.

Mobile-responsive: Must remain fully responsive using the default Docusaurus theme.

No manual editing of generated folders (e.g., .docusaurus, build output, etc.).

Chapters: Minimum 5 fully completed chapters, each created from its own spec.

Success Criteria

The site is live at:
https://<username>.github.io/<repo>/

npm ci && npm run build runs with zero errors on a fresh clone.

Lighthouse scores:

Performance ≥ 90

Accessibility ≥ 95

All chapters trace directly back to corresponding .spec.md files.

All code blocks execute correctly when applicable.

All deployment actions succeed on main branch.

Entire workflow remains reproducible with no untracked changes.

All writing passes clarity checks (Flesch-Kincaid grade ~10–12 recommended for readability).

AI Guidelines

AI tools (e.g., Claude Code) must never create content without referencing a specific approved spec.

AI must refuse requests that violate this constitution.

When uncertain or ambiguous → AI must propose a new spec instead of guessing.

All generated content must embed a comment linking back to its creating spec.

## Response snapshot

The project constitution has been updated to version 1.0.0. This is a MAJOR version bump due to the addition of all core principles, key standards, constraints, and governance sections which were previously undefined.

**Files flagged for manual follow-up:**
- `README.md`: Needs to be updated to reflect the new principles and guidelines.

**Suggested commit message:**
docs: amend constitution to v1.0.0 (initial principles, standards, constraints, and governance)

## Outcome

- ✅ Impact: Constitution updated, project guidelines established.
- 🧪 Tests: N/A
- 📁 Files: 1 file modified.
- 🔁 Next prompts: N/A
- 🧠 Reflection: Successfully updated the constitution based on user input, ensuring all principles, standards, constraints, and governance are documented.

## Evaluation notes (flywheel)

- Failure modes observed: N/A
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
