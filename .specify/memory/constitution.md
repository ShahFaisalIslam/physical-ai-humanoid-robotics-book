<!-- Sync Impact Report:
Version change: N/A (initial) → 0.1.0
Modified principles: None (all new)
Added sections: Mission Statement, Core Principles (Pedagogical Foundations, Structural Integrity, Technical & Development Philosophy), Content & RAG Constraints, Content Boundaries, Quality Standards, Success Criteria, Governance
Removed sections: None
Templates requiring updates:
  - .specify/templates/plan-template.md: ✅ updated
  - .specify/templates/spec-template.md: ✅ updated
  - .specify/templates/tasks-template.md: ✅ updated
  - .specify/templates/commands/*.md: ✅ updated
Follow-up TODOs: None
-->
## Constitution: Physical AI & Humanoid Robotics Course Book

This Constitution governs the development of the course book, ensuring strict adherence to the **spec-driven development methodology** and all stated project constraints.

### 1. Mission Statement

This book is the **authoritative instructional reference** for the *Physical AI & Humanoid Robotics* course. Its purpose is to define **what must be taught, why it matters, and the non-negotiable pedagogical structure** over a **15-week quarter**, translating established AI Native Development (AID) principles into verifiable Physical AI systems.

### 2. Core Principles

#### 2.1 Pedagogical Foundations (Instructor Primary Audience)

* **Primary Focus: Instructor Clarity:** Content priority is placed on guiding the instructor on **what to emphasize, what to skip, and how concepts map to assessments and learning outcomes**. Student-facing instructions exist to support the instructor's delivery, not as the final product.
* **Continuity Over Novelty:** New physical concepts must be explicitly framed as an **extension of an existing AID concept**. This reinforces the bridge between software-centric agents and embodied intelligence.
* **Authoritative Guidance:** The book must present **curated, opinionated conclusions** and single best practices. Trade-offs may be acknowledged, but the content must be definitive and instructive, minimizing ambiguity for teaching delivery.

#### 2.2 Structural Integrity (Temporal & Scope Constraints)

* **Strict Temporal Constraint:** The book must align with a **15-week quarter**. It will contain **13 core weekly instructional units/chapters** to cover the mandatory content, reserving weeks 14 and 15 for **contingency, review, and final assessment/integration**.
    * *Constraint:* The weekly breakdown must be strictly adhered to; content cannot be reordered without a major edition update (X.0.0).
* **Weekly Breakdown Adherence:** The book's structure must **strictly adhere** to the course's mandated weekly breakdown, not imposing arbitrary structural grouping (e.g., Parts or Modules) unless necessary to support the RAG constraint.
* **Project-First Philosophy:** Theory is delivered **just-in-time** to enable the next component of the cumulative, final demonstrable physical AI system. Every week contributes a tangible component.

#### 2.3 Technical & Development Philosophy

* **Specification Discipline:** Tool selection, architectural diagrams, and specific content breakdown must be reserved for the **Planning** and **Specification** phases, respectively. This Constitution only governs the *intent* and *constraints*.
* **Hardware-Centric Reality:** The course is **explicitly tied** to the specific recommended hardware. Hardware constraints are treated as **design inputs**, not inconveniences.
* **Open-Source & Free-Tier Bias:** Prioritize solutions compatible with students' existing free-tier orientation and favor open-source tools/frameworks.

### 3. Content & RAG Constraints (Non-Negotiable)

The following constraints are mandatory to ensure the future functionality of the embedded RAG chatbot:

* **RAG-Optimized Modularity:** All content must be structured using explicit, hierarchical Markdown headings (e.g., `##`, `###`) to create **clearly defined, single-topic segments**. Ambiguous or excessively long sections are prohibited as they degrade RAG vector indexing performance.
* **Explicit Terminology:** All jargon (Physical AI, AID, or Robotics) must be **defined precisely** upon its first use in the book. A dedicated glossary must be included in the Specification phase.
* **Table Isolation:** All provided **hardware requirements, tables, and specifications** must be documented in a dedicated, separate reference section (e.g., an Appendix) to ensure they are easily referenced by the RAG system without confusing the core instructional narrative flow.
* **No Redundancy:** Content must be concise, avoiding repeating explanations across sections, which inflates the vector space and can introduce conflicting RAG responses.

### 4. Content Boundaries

#### 4.1 What This Book IS

* An **instructional anchor** for teaching the course.
* A **15-week structural commitment** to cover the material.
* A **bridge document** showing the translation of AID skills to embodied systems.
* A **reference manual** for the specific hardware/software stack.

#### 4.2 What This Book IS NOT

* Not a generic robotics textbook or a theoretical computer science reference.
* Not a word-for-word teaching script for live instruction.
* Not hardware-agnostic; it is deliberately tied to the specified platform.
* Not designed for self-study without the corresponding course structure.

### 5. Quality Standards

#### 5.1 Content Requirements

* **Every unit must explicitly map to a Learning Outcome/Assessment.** No "nice-to-have" content is permitted.
* **Clear Connection Points:** Each week must start with an explicit tie-in showing the instructor how to connect the new physical concept to a prior AID concept.
* **Validation:** Every practical exercise or build component must include **clear validation criteria** to verify functionality.

#### 5.2 Documentation & Safety

* **Safety First:** Given the Physical AI context, mandatory, non-negotiable **safety warnings** must accompany any hardware manipulation or power-related steps.
* **Clarity for AI Native Graduates:** Assume a high level of programming competency but **zero prior physical AI/robotics experience**. The tone must be direct and practical.

### 6. Success Criteria

**This book succeeds if:**

1.  **Instructor Consensus:** Multiple course instructors can use the book to deliver the course with **high pedagogical consistency**.
2.  **Assessment Alignment:** All course assessments are directly and exclusively supported by the content of the book.
3.  **RAG Readiness:** The content structure, formatting, and terminology allow for the successful, high-accuracy deployment of the embedded RAG chatbot.
4.  **Final Demonstration:** The cumulative project results in a demonstrable, functional physical AI system by the end of Week 13.

**This book fails if:**

* Structural choices (e.g., lack of clear headings, ambiguity) inhibit RAG performance.
* The weekly pacing proves unrealistic for the 13 weeks of content.
* Instructors deviate significantly from the recommended structure or content delivery.

## Governance
**Version**: 0.1.0 | **Ratified**: 2025-12-15 | **Last Amended**: 2025-12-15
