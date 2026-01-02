---
id: 2
title: "AI Spec-Driven Book Creation Constitution Update"
stage: constitution
date_iso: "2026-01-01"
surface: agent
model: claude-sonnet-4-5-20250929
feature: "none"
branch: "master"
user: "user"
command: "/sp.constitution"
labels: ["constitution", "spec-driven", "ai-book", "documentation"]
files_yaml: |
  - .specify/memory/constitution.md
  - history/prompts/constitution/2-ai-spec-driven-book-creation-constitution-update.constitution.prompt.md
tests_yaml: |
  - "none"
links: |
  spec: null
  ticket: null
  adr: null
  pr: null
---

## Prompt Text
```
/sp.constitution

Project: AI/Spec-Driven Unified Book Creation using Spec-Kit Plus and Claude Code

Objective:
To design, write, and deploy a complete, production-quality technical book using AI-native software development practices, powered by Spec-Kit Plus and Claude Code, and published through Docusaurus on GitHub Pages.

Core Principles:

* Accuracy through direct verification of primary and trusted technical sources
* Clarity and accessibility for a technical audience (students, developers, and AI engineers)
* AI-native development with human-in-the-loop validation
* Reproducibility of content, structure, and workflow
* Transparency of sources, tools, and generation process
* Continuous improvement through iterative refinement

Key Standards:

* All factual and technical claims must be verifiable through credible sources
* All explanations must be beginner-friendly while remaining technically correct
* AI-generated content must be reviewed, edited, and validated by humans
* Code examples must be runnable, minimal, and clearly explained
* Citations must be consistently formatted
* No hallucinated concepts, tools, or references are allowed
* All chapters must follow a consistent documentation structure defined by Spec-Kit Plus

Source & Citation Rules:

* Minimum 60% sources from official documentation, research papers, or authoritative technical platforms
* Remaining sources may include trusted blogs, GitHub repositories, and industry case studies
* Plagiarism tolerance: 0%
* All reused ideas must be rewritten originally with proper attribution

Writing & Content Quality:

* Language: Clear, precise, and structured
* Tone: Professional, educational, and practical
* Target readability: Intermediate technical level
* Each chapter must include:

  * Concept explanation
  * Practical example
  * Use case or real-world application
  * Summary

Technical Constraints:

* Platform: Docusaurus
* Deployment: GitHub Pages
* Tooling:

  * Spec-Kit Plus for specification-driven structure
  * Claude Code for AI-assisted content generation
* Version control must be maintained via GitHub
* All content must be written in Markdown
* The book must build successfully with zero build errors

Project Constraints:

* Book length: Determined by topic depth, no artificial padding
* Chapters must be modular and independently readable
* No copyrighted material may be copied directly
* All diagrams must be either original or properly attributed

Success Criteria:

* Fully deployed live book on GitHub Pages
* All chapters pass human factual verification
* Zero plagiarism detected
* Clean GitHub repository with documented workflow
* Clear demonstration of AI-native software development using Spec-driven processes
* Judges can reproduce the book-generation process using the provided specs

Ethical & AI Usage Guidelines:

* AI is used as a co-author, not a replacement for human judgment
* All misleading, biased, or unverifiable content must be removed
* Safety, privacy, and responsible AI principles must be respected at all stages

Final Acceptance Conditions:

* The book builds successfully
* The deployed site is publicly accessible
* The content reflects real understanding, not just AI output
* The project clearly demonstrates the power of Spec-Kit Plus + Claude Code working together
*
```

## Response Text
Updated the constitution document to reflect the new "AI/Spec-Driven Unified Book Creation using Spec-Kit Plus and Claude Code" project. The previous Physical AI book constitution was replaced with a comprehensive constitution that includes:

- Vision: Designing, writing, and deploying a complete technical book using AI-native software development practices
- Core Principles: Accuracy through verification, clarity and accessibility, AI-native development with human validation, reproducibility and transparency, technical excellence, and ethical content creation
- Success Criteria: Deployment on GitHub Pages, human verification, zero plagiarism, clean repository, reproducible process, and demonstration of Spec-Kit Plus + Claude Code integration
- Constraints: Topic-driven length, modular chapters, original content, proper attribution, consistent documentation structure
- Stakeholders: Technical audience, judges, and open source community
- Brand Voice: Professional, educational, and practical

The constitution has been saved to .specify/memory/constitution.md with version 2.0.0 (incremented due to major content changes), ratified on 2026-01-01.

## Outcome
Successfully updated the constitution document to align with the new AI/Spec-Driven Unified Book Creation project requirements.

## Evaluation
The updated constitution properly addresses all required elements from the new project specification, including all core principles, success criteria, constraints, and ethical guidelines. The version was properly incremented from 1.0.0 to 2.0.0 to reflect the major content changes from a Physical AI book to an AI/Spec-Driven book creation project.