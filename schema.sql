PRAGMA foreign_keys = ON;

CREATE TABLE IF NOT EXISTS nodes (
    id TEXT PRIMARY KEY,
    type TEXT NOT NULL CHECK (type IN ('TASK','STRATEGY','CAPABILITY','SKILL')),
    name TEXT NOT NULL,
    description TEXT NOT NULL DEFAULT '',
    spec_json TEXT NOT NULL DEFAULT '{}',
    status TEXT NOT NULL DEFAULT 'proposed',
    created_at TEXT NOT NULL
);

CREATE TABLE IF NOT EXISTS edges (
    source TEXT NOT NULL,
    target TEXT NOT NULL,
    relation TEXT NOT NULL CHECK (relation IN ('HAS_STRATEGY','REQUIRES','IMPLEMENTS')),
    metadata_json TEXT NOT NULL DEFAULT '{}',
    PRIMARY KEY (source, target, relation),
    FOREIGN KEY (source) REFERENCES nodes(id) ON DELETE CASCADE,
    FOREIGN KEY (target) REFERENCES nodes(id) ON DELETE CASCADE
);

CREATE TABLE IF NOT EXISTS skill_versions (
    id TEXT PRIMARY KEY,
    capability_ids TEXT NOT NULL DEFAULT '[]',
    version TEXT NOT NULL DEFAULT 'v1',
    code_path TEXT NOT NULL,
    source_hash TEXT NOT NULL,
    kwargs_json TEXT NOT NULL DEFAULT '{}',
    contract_json TEXT NOT NULL DEFAULT '{}',
    status TEXT NOT NULL DEFAULT 'candidate',
    created_at TEXT NOT NULL
);

CREATE TABLE IF NOT EXISTS validations (
    id TEXT PRIMARY KEY,
    skill_id TEXT NOT NULL,
    version TEXT NOT NULL,
    scenario_set TEXT NOT NULL DEFAULT '',
    seed_domain TEXT NOT NULL DEFAULT '',
    metrics_json TEXT NOT NULL DEFAULT '{}',
    evidence_path TEXT NOT NULL DEFAULT '',
    verdict TEXT NOT NULL,
    created_at TEXT NOT NULL,
    FOREIGN KEY (skill_id) REFERENCES nodes(id) ON DELETE CASCADE
);

CREATE TABLE IF NOT EXISTS experiments (
    id TEXT PRIMARY KEY,
    job_type TEXT NOT NULL,
    capability_id TEXT,
    actor_id TEXT NOT NULL DEFAULT '',
    budget REAL NOT NULL DEFAULT 0,
    measured_cost REAL NOT NULL DEFAULT 0,
    result_json TEXT NOT NULL DEFAULT '{}',
    created_at TEXT NOT NULL
);

CREATE TABLE IF NOT EXISTS execution_plans (
    id TEXT PRIMARY KEY,
    task_id TEXT NOT NULL,
    strategy_id TEXT NOT NULL,
    steps_json TEXT NOT NULL DEFAULT '[]',
    status TEXT NOT NULL DEFAULT 'draft',
    created_at TEXT NOT NULL
);

CREATE TABLE IF NOT EXISTS state (
    key TEXT PRIMARY KEY,
    value TEXT NOT NULL
);

CREATE INDEX IF NOT EXISTS idx_nodes_type ON nodes(type);
CREATE INDEX IF NOT EXISTS idx_nodes_name ON nodes(name);
CREATE INDEX IF NOT EXISTS idx_edges_source ON edges(source);
CREATE INDEX IF NOT EXISTS idx_edges_target ON edges(target);
CREATE INDEX IF NOT EXISTS idx_skill_versions_status ON skill_versions(status);
