-- ARIAC Competition Database Schema
-- SQLite database for tracking competitors, trials, and runs

-- Drop tables if they exist (for clean recreation)
DROP TABLE IF EXISTS OrderSubmission;
DROP TABLE IF EXISTS Penalty;
DROP TABLE IF EXISTS Run;
DROP TABLE IF EXISTS Trial;
DROP TABLE IF EXISTS Competitor;
DROP VIEW IF EXISTS competition_results;

-- Create Competitor table
CREATE TABLE Competitor (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    name TEXT NOT NULL
);

-- Create Trial table
CREATE TABLE Trial (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    trial_id TEXT NOT NULL,
    config_hash TEXT,
    seed INTEGER,
    time_limit INTEGER,
    num_kits INTEGER,
    num_modules INTEGER,
    num_high_priority INTEGER
);

-- Create Run table
CREATE TABLE Run (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    trial_id INTEGER NOT NULL,
    competitor_id INTEGER NOT NULL,
    completed BOOLEAN DEFAULT FALSE,
    aborted BOOLEAN DEFAULT FALSE,
    sensor_cost REAL DEFAULT 0.0,
    duration REAL DEFAULT 0.0,
    total_cells INTEGER DEFAULT 0,
    defective_cells INTEGER DEFAULT 0,
    avg_report_time REAL DEFAULT 0.0,
    num_reports_submitted INTEGER DEFAULT 0,
    num_correct_reports INTEGER DEFAULT 0,
    num_correct_report_classifications INTEGER DEFAULT 0,
    FOREIGN KEY (trial_id) REFERENCES Trial(id),
    FOREIGN KEY (competitor_id) REFERENCES Competitor(id)
);

-- Create Penalty table
CREATE TABLE Penalty (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    run_id INTEGER NOT NULL,
    type INTEGER NOT NULL,
    description TEXT,
    time REAL,
    FOREIGN KEY (run_id) REFERENCES Run(id)
);

-- Create OrderSubmission table
CREATE TABLE OrderSubmission (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    run_id INTEGER NOT NULL,
    submission_time REAL NOT NULL,
    announcement_time REAL,
    time_limit REAL,
    order_type INTEGER,
    FOREIGN KEY (run_id) REFERENCES Run(id)
);

-- Create indexes for better performance
CREATE INDEX idx_run_trial_id ON Run(trial_id);
CREATE INDEX idx_run_competitor_id ON Run(competitor_id);
CREATE INDEX idx_trial_trial_id ON Trial(trial_id);
CREATE INDEX idx_penalty_run_id ON Penalty(run_id);
CREATE INDEX idx_order_submission_run_id ON OrderSubmission(run_id);

-- Useful queries for analysis

-- View to join all data for easy analysis
CREATE VIEW competition_results AS
SELECT 
    c.name AS competitor_name,
    t.trial_id,
    t.config_hash,
    t.seed,
    t.time_limit,
    t.num_kits AS total_kits,
    t.num_modules AS total_modules,
    r.completed,
    r.aborted,
    r.sensor_cost,
    r.duration,
    r.total_cells,
    r.defective_cells,
    r.avg_report_time,
    r.num_reports_submitted,
    r.num_correct_reports,
    r.num_correct_report_classifications,
    -- Calculate quality metrics
    ROUND((r.defective_cells * 100.0 / NULLIF(r.total_cells, 0)), 2) AS defective_rate_percentage,
    ROUND((r.num_correct_reports * 100.0 / NULLIF(r.num_reports_submitted, 0)), 2) AS report_accuracy_percentage,
    ROUND((r.num_correct_report_classifications * 100.0 / NULLIF(r.num_reports_submitted, 0)), 2) AS classification_accuracy_percentage,
    -- Calculate penalty count
    (SELECT COUNT(*) FROM Penalty p WHERE p.run_id = r.id) AS penalty_count,
    -- Calculate order submission count
    (SELECT COUNT(*) FROM OrderSubmission os WHERE os.run_id = r.id) AS order_submission_count
FROM Run r
JOIN Competitor c ON r.competitor_id = c.id
JOIN Trial t ON r.trial_id = t.id;
