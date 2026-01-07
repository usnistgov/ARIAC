#include <db_manager/db_manager.hpp>

#include <algorithm>
#include <fstream>
#include <functional>
#include <sstream>

namespace ariac_db {

// OrderType utility functions
std::string orderTypeToString(OrderType type) {
    switch (type) {
        case OrderType::KIT:
            return "Assembly";
        case OrderType::MODULE:
            return "Kitting";
        case OrderType::HIGH_PRIORITY:
            return "Combined";
        case OrderType::UNKNOWN:
        default:
            return "Unknown";
    }
}

OrderType intToOrderType(int type) {
    switch (type) {
        case 1:
            return OrderType::KIT;
        case 2:
            return OrderType::MODULE;
        case 3:
            return OrderType::HIGH_PRIORITY;
        default:
            return OrderType::UNKNOWN;
    }
}

int orderTypeToInt(OrderType type) {
    switch (type) {
        case OrderType::KIT:
            return 1;
        case OrderType::MODULE:
            return 2;
        case OrderType::HIGH_PRIORITY:
            return 3;
        case OrderType::UNKNOWN:
        default:
            return -1;
    }
}

DatabaseManager::DatabaseManager(const std::string& db_path)
    : db_path_(db_path), db_(nullptr), connected_(false) {
    ARIAC_DB_LOG("DatabaseManager constructor: " << db_path_);
}

DatabaseManager::~DatabaseManager() {
    disconnect();
    ARIAC_DB_LOG("DatabaseManager destructor");
}

DatabaseManager::DatabaseManager(DatabaseManager&& other) noexcept
    : db_path_(std::move(other.db_path_)), db_(other.db_), connected_(other.connected_), last_error_(std::move(other.last_error_)) {
    other.db_ = nullptr;
    other.connected_ = false;
}

DatabaseManager& DatabaseManager::operator=(DatabaseManager&& other) noexcept {
    if (this != &other) {
        disconnect();

        db_path_ = std::move(other.db_path_);
        db_ = other.db_;
        connected_ = other.connected_;
        last_error_ = std::move(other.last_error_);

        other.db_ = nullptr;
        other.connected_ = false;
    }
    return *this;
}

bool DatabaseManager::connect() {
    std::lock_guard<std::mutex> lock(db_mutex_);

    if (connected_) {
        ARIAC_DB_LOG("Already connected to database");
        return true;
    }

    // Check if database file exists
    std::ifstream file(db_path_);
    if (!file.good()) {
        setLastError("Database file does not exist: " + db_path_);
        ARIAC_DB_ERROR("Database file not found: " + db_path_);
        return false;
    }
    file.close();

    ARIAC_DB_LOG("Attempting to connect to database: " + db_path_);

    int result = sqlite3_open(db_path_.c_str(), &db_);
    if (result != SQLITE_OK) {
        std::string error_msg = sqlite3_errmsg(db_);
        setLastError("Failed to open database: " + error_msg);
        ARIAC_DB_ERROR("SQLite open failed: " + error_msg);
        if (db_) {
            sqlite3_close(db_);
            db_ = nullptr;
        }
        return false;
    }

    ARIAC_DB_LOG("Database opened successfully");

    // Test basic connectivity
    char* error_msg = nullptr;
    result = sqlite3_exec(db_, "SELECT 1;", nullptr, nullptr, &error_msg);
    if (result != SQLITE_OK) {
        std::string err = error_msg ? std::string(error_msg) : "Unknown error";
        setLastError("Database connectivity test failed: " + err);
        ARIAC_DB_ERROR("Connectivity test failed: " + err);
        if (error_msg)
            sqlite3_free(error_msg);
        sqlite3_close(db_);
        db_ = nullptr;
        return false;
    }

    ARIAC_DB_LOG("Database connectivity test passed");

    // Try to enable foreign key constraints (non-critical)
    result = sqlite3_exec(db_, "PRAGMA foreign_keys = ON;", nullptr, nullptr, &error_msg);
    if (result != SQLITE_OK) {
        std::string err = error_msg ? std::string(error_msg) : "Unknown error";
        ARIAC_DB_LOG("Warning: Could not enable foreign key constraints: " + err);
        // Don't fail connection for this - it's not critical
        if (error_msg)
            sqlite3_free(error_msg);
    } else {
        ARIAC_DB_LOG("Foreign key constraints enabled");
    }

    // Validate schema (non-critical)
    if (!validateSchema()) {
        ARIAC_DB_LOG("Warning: Schema validation failed, but continuing anyway");
        // Don't fail connection - schema might be different but still work
    } else {
        ARIAC_DB_LOG("Schema validation passed");
    }

    connected_ = true;
    ARIAC_DB_LOG("Successfully connected to database: " + db_path_);
    return true;
}

void DatabaseManager::disconnect() {
    std::lock_guard<std::mutex> lock(db_mutex_);

    if (db_) {
        sqlite3_close(db_);
        db_ = nullptr;
    }
    connected_ = false;
    ARIAC_DB_LOG("Disconnected from database");
}

bool DatabaseManager::isConnected() const {
    std::lock_guard<std::mutex> lock(db_mutex_);
    return connected_;
}

std::string DatabaseManager::getLastError() const {
    std::lock_guard<std::mutex> lock(db_mutex_);
    return last_error_;
}

int DatabaseManager::insertRun(int trial_id, int competitor_id, const RunData& data) {
    std::lock_guard<std::mutex> lock(db_mutex_);

    if (!connected_) {
        setLastError("Database not connected");
        return -1;
    }

    const char* sql = R"(
         INSERT INTO Run (trial_id, competitor_id, completed, aborted, sensor_cost, 
                         duration, total_cells, defective_cells, avg_report_time, 
                         num_reports_submitted, num_correct_reports, num_correct_report_classifications)
         VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
     )";

    sqlite3_stmt* stmt;
    int result = sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr);
    if (result != SQLITE_OK) {
        setLastError("Failed to prepare insert statement: " + std::string(sqlite3_errmsg(db_)));
        return -1;
    }

    // Bind parameters
    bindParameter(stmt, 1, trial_id);
    bindParameter(stmt, 2, competitor_id);
    bindParameter(stmt, 3, data.completed);
    bindParameter(stmt, 4, data.aborted);
    bindParameter(stmt, 5, data.sensor_cost);
    bindParameter(stmt, 6, data.duration);
    bindParameter(stmt, 7, data.total_cells);
    bindParameter(stmt, 8, data.defective_cells);
    bindParameter(stmt, 9, data.avg_report_time);
    bindParameter(stmt, 10, data.num_reports_submitted);
    bindParameter(stmt, 11, data.num_correct_reports);
    bindParameter(stmt, 12, data.num_correct_report_classifications);

    result = sqlite3_step(stmt);
    int run_id = -1;

    if (result == SQLITE_DONE) {
        run_id = static_cast<int>(sqlite3_last_insert_rowid(db_));
        ARIAC_DB_LOG("Inserted run with ID: " << run_id);
    } else {
        setLastError("Failed to insert run: " + std::string(sqlite3_errmsg(db_)));
    }

    sqlite3_finalize(stmt);
    return run_id;
}

int DatabaseManager::insertRunForTrial(const std::string& trial_id_str, int competitor_id, const RunData& data) {
    // First get the trial ID from the trial identifier string
    int trial_id = getTrialId(trial_id_str);
    if (trial_id == -1) {
        setLastError("Trial not found: " + trial_id_str);
        return -1;
    }
    
    // Now insert the run with the found trial ID
    return insertRun(trial_id, competitor_id, data);
}

bool DatabaseManager::updateRun(int run_id, const RunData& data) {
    std::lock_guard<std::mutex> lock(db_mutex_);

    if (!connected_) {
        setLastError("Database not connected");
        return false;
    }

    const char* sql = R"(
         UPDATE Run SET completed = ?, aborted = ?, sensor_cost = ?, duration = ?,
                       total_cells = ?, defective_cells = ?, avg_report_time = ?,
                       num_reports_submitted = ?, num_correct_reports = ?, 
                       num_correct_report_classifications = ?
         WHERE id = ?
     )";

    sqlite3_stmt* stmt;
    int result = sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr);
    if (result != SQLITE_OK) {
        setLastError("Failed to prepare update statement: " + std::string(sqlite3_errmsg(db_)));
        return false;
    }

    // Bind parameters
    bindParameter(stmt, 1, data.completed);
    bindParameter(stmt, 2, data.aborted);
    bindParameter(stmt, 3, data.sensor_cost);
    bindParameter(stmt, 4, data.duration);
    bindParameter(stmt, 5, data.total_cells);
    bindParameter(stmt, 6, data.defective_cells);
    bindParameter(stmt, 7, data.avg_report_time);
    bindParameter(stmt, 8, data.num_reports_submitted);
    bindParameter(stmt, 9, data.num_correct_reports);
    bindParameter(stmt, 10, data.num_correct_report_classifications);
    bindParameter(stmt, 11, run_id);

    result = sqlite3_step(stmt);
    bool success = (result == SQLITE_DONE);

    if (!success) {
        setLastError("Failed to update run: " + std::string(sqlite3_errmsg(db_)));
    }

    sqlite3_finalize(stmt);
    return success;
}

std::vector<RunData> DatabaseManager::getRuns(const std::string& where_clause) {
    std::lock_guard<std::mutex> lock(db_mutex_);

    std::vector<RunData> runs;

    if (!connected_) {
        setLastError("Database not connected");
        return runs;
    }

    std::string sql =
        "SELECT id, trial_id, competitor_id, completed, aborted, sensor_cost, "
        "duration, total_cells, defective_cells, avg_report_time, "
        "num_reports_submitted, num_correct_reports, num_correct_report_classifications "
        "FROM Run";

    if (!where_clause.empty()) {
        sql += " WHERE " + where_clause;
    }

    sql += " ORDER BY id DESC";

    sqlite3_stmt* stmt;
    int result = sqlite3_prepare_v2(db_, sql.c_str(), -1, &stmt, nullptr);
    if (result != SQLITE_OK) {
        setLastError("Failed to prepare select statement: " + std::string(sqlite3_errmsg(db_)));
        return runs;
    }

    while ((result = sqlite3_step(stmt)) == SQLITE_ROW) {
        RunData run;
        run.id = getColumnInt(stmt, 0);
        run.trial_id = getColumnInt(stmt, 1);
        run.competitor_id = getColumnInt(stmt, 2);
        run.completed = getColumnBool(stmt, 3);
        run.aborted = getColumnBool(stmt, 4);
        run.sensor_cost = getColumnDouble(stmt, 5);
        run.duration = getColumnDouble(stmt, 6);
        run.total_cells = getColumnInt(stmt, 7);
        run.defective_cells = getColumnInt(stmt, 8);
        run.avg_report_time = getColumnDouble(stmt, 9);
        run.num_reports_submitted = getColumnInt(stmt, 10);
        run.num_correct_reports = getColumnInt(stmt, 11);
        run.num_correct_report_classifications = getColumnInt(stmt, 12);

        runs.push_back(run);
    }

    if (result != SQLITE_DONE) {
        setLastError("Error reading runs: " + std::string(sqlite3_errmsg(db_)));
        runs.clear();
    }

    sqlite3_finalize(stmt);
    ARIAC_DB_LOG("Retrieved " << runs.size() << " runs");
    return runs;
}

RunData DatabaseManager::getRun(int run_id) {
    auto runs = getRuns("id = " + std::to_string(run_id));
    if (runs.empty()) {
        RunData empty_run;
        empty_run.id = -1;
        return empty_run;
    }
    return runs[0];
}

std::vector<RunData> DatabaseManager::getRunsByTeam(const std::string& team_name) {
    int competitor_id = getCompetitorId(team_name);
    if (competitor_id == -1) {
        return std::vector<RunData>();
    }

    return getRuns("competitor_id = " + std::to_string(competitor_id));
}

std::vector<RunData> DatabaseManager::getRunsByTrial(const std::string& trial_id) {
    int trial_pk = getTrialId(trial_id);
    if (trial_pk == -1) {
        return std::vector<RunData>();
    }

    return getRuns("trial_id = " + std::to_string(trial_pk));
}

bool DatabaseManager::runExists(int run_id) {
    std::lock_guard<std::mutex> lock(db_mutex_);

    if (!connected_) {
        setLastError("Database not connected");
        return false;
    }

    const char* sql = "SELECT COUNT(*) FROM Run WHERE id = ?";

    sqlite3_stmt* stmt;
    int result = sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr);
    if (result != SQLITE_OK) {
        setLastError("Failed to prepare run exists query: " + std::string(sqlite3_errmsg(db_)));
        return false;
    }

    bindParameter(stmt, 1, run_id);

    bool exists = false;
    if (sqlite3_step(stmt) == SQLITE_ROW) {
        int count = getColumnInt(stmt, 0);
        exists = (count > 0);
    }

    sqlite3_finalize(stmt);
    return exists;
}

int DatabaseManager::getCompetitorId(const std::string& team_name) {
    std::lock_guard<std::mutex> lock(db_mutex_);

    if (!connected_) {
        setLastError("Database not connected");
        return -1;
    }

    const char* sql = "SELECT id FROM Competitor WHERE name = ?";

    sqlite3_stmt* stmt;
    int result = sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr);
    if (result != SQLITE_OK) {
        setLastError("Failed to prepare competitor query: " + std::string(sqlite3_errmsg(db_)));
        return -1;
    }

    bindParameter(stmt, 1, team_name);

    int competitor_id = -1;
    if (sqlite3_step(stmt) == SQLITE_ROW) {
        competitor_id = getColumnInt(stmt, 0);
    }

    sqlite3_finalize(stmt);
    return competitor_id;
}

CompetitorData DatabaseManager::getCompetitor(const std::string& team_name) {
    std::lock_guard<std::mutex> lock(db_mutex_);

    CompetitorData competitor;
    competitor.id = -1;

    if (!connected_) {
        setLastError("Database not connected");
        return competitor;
    }

    const char* sql = "SELECT id, name FROM Competitor WHERE name = ?";

    sqlite3_stmt* stmt;
    int result = sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr);
    if (result != SQLITE_OK) {
        setLastError("Failed to prepare competitor query: " + std::string(sqlite3_errmsg(db_)));
        return competitor;
    }

    bindParameter(stmt, 1, team_name);

    if (sqlite3_step(stmt) == SQLITE_ROW) {
        competitor.id = getColumnInt(stmt, 0);
        competitor.name = getColumnText(stmt, 1);
    }

    sqlite3_finalize(stmt);
    return competitor;
}

std::vector<CompetitorData> DatabaseManager::getAllCompetitors() {
    std::lock_guard<std::mutex> lock(db_mutex_);

    std::vector<CompetitorData> competitors;

    if (!connected_) {
        setLastError("Database not connected");
        return competitors;
    }

    const char* sql = "SELECT id, name FROM Competitor ORDER BY name";

    sqlite3_stmt* stmt;
    int result = sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr);
    if (result != SQLITE_OK) {
        setLastError("Failed to prepare competitors query: " + std::string(sqlite3_errmsg(db_)));
        return competitors;
    }

    while ((result = sqlite3_step(stmt)) == SQLITE_ROW) {
        CompetitorData competitor;
        competitor.id = getColumnInt(stmt, 0);
        competitor.name = getColumnText(stmt, 1);
        competitors.push_back(competitor);
    }

    sqlite3_finalize(stmt);
    return competitors;
}

int DatabaseManager::insertCompetitor(const std::string& team_name) {
    std::lock_guard<std::mutex> lock(db_mutex_);

    if (!connected_) {
        setLastError("Database not connected");
        return -1;
    }

    const char* sql = "INSERT INTO Competitor (name) VALUES (?)";

    sqlite3_stmt* stmt;
    int result = sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr);
    if (result != SQLITE_OK) {
        setLastError("Failed to prepare insert competitor statement: " + std::string(sqlite3_errmsg(db_)));
        return -1;
    }

    bindParameter(stmt, 1, team_name);

    result = sqlite3_step(stmt);
    int competitor_id = -1;

    if (result == SQLITE_DONE) {
        competitor_id = static_cast<int>(sqlite3_last_insert_rowid(db_));
        ARIAC_DB_LOG("Inserted competitor '" << team_name << "' with ID: " << competitor_id);
    } else {
        setLastError("Failed to insert competitor: " + std::string(sqlite3_errmsg(db_)));
    }

    sqlite3_finalize(stmt);
    return competitor_id;
}

int DatabaseManager::getTrialId(const std::string& trial_id) {
    std::lock_guard<std::mutex> lock(db_mutex_);

    if (!connected_) {
        setLastError("Database not connected");
        return -1;
    }

    const char* sql = "SELECT id FROM Trial WHERE trial_id = ?";

    sqlite3_stmt* stmt;
    int result = sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr);
    if (result != SQLITE_OK) {
        setLastError("Failed to prepare trial query: " + std::string(sqlite3_errmsg(db_)));
        return -1;
    }

    bindParameter(stmt, 1, trial_id);

    int trial_pk = -1;
    if (sqlite3_step(stmt) == SQLITE_ROW) {
        trial_pk = getColumnInt(stmt, 0);
    }

    sqlite3_finalize(stmt);
    return trial_pk;
}

TrialData DatabaseManager::getTrial(const std::string& trial_id) {
    std::lock_guard<std::mutex> lock(db_mutex_);

    TrialData trial;
    trial.id = -1;

    if (!connected_) {
        setLastError("Database not connected");
        return trial;
    }

    const char* sql =
        "SELECT id, trial_id, config_hash, seed, time_limit, num_kits, num_modules, num_high_priority "
        "FROM Trial WHERE trial_id = ?";

    sqlite3_stmt* stmt;
    int result = sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr);
    if (result != SQLITE_OK) {
        setLastError("Failed to prepare trial query: " + std::string(sqlite3_errmsg(db_)));
        return trial;
    }

    bindParameter(stmt, 1, trial_id);

    if (sqlite3_step(stmt) == SQLITE_ROW) {
        trial.id = getColumnInt(stmt, 0);
        trial.trial_id = getColumnText(stmt, 1);
        trial.config_hash = getColumnText(stmt, 2);
        trial.seed = getColumnInt(stmt, 3);
        trial.time_limit = getColumnInt(stmt, 4);
        trial.num_kits = getColumnInt(stmt, 5);
        trial.num_modules = getColumnInt(stmt, 6);
        trial.num_high_priority = getColumnInt(stmt, 7);
    }

    sqlite3_finalize(stmt);
    return trial;
}

std::vector<TrialData> DatabaseManager::getAllTrials() {
    std::lock_guard<std::mutex> lock(db_mutex_);

    std::vector<TrialData> trials;

    if (!connected_) {
        setLastError("Database not connected");
        return trials;
    }

    const char* sql =
        "SELECT id, trial_id, config_hash, seed, time_limit, num_kits, num_modules, num_high_priority "
        "FROM Trial ORDER BY trial_id";

    sqlite3_stmt* stmt;
    int result = sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr);
    if (result != SQLITE_OK) {
        setLastError("Failed to prepare trials query: " + std::string(sqlite3_errmsg(db_)));
        return trials;
    }

    while ((result = sqlite3_step(stmt)) == SQLITE_ROW) {
        TrialData trial;
        trial.id = getColumnInt(stmt, 0);
        trial.trial_id = getColumnText(stmt, 1);
        trial.config_hash = getColumnText(stmt, 2);
        trial.seed = getColumnInt(stmt, 3);
        trial.time_limit = getColumnInt(stmt, 4);
        trial.num_kits = getColumnInt(stmt, 5);
        trial.num_modules = getColumnInt(stmt, 6);
        trial.num_high_priority = getColumnInt(stmt, 7);
        trials.push_back(trial);
    }

    sqlite3_finalize(stmt);
    return trials;
}

int DatabaseManager::insertTrial(const TrialData& trial_data) {
    std::lock_guard<std::mutex> lock(db_mutex_);

    if (!connected_) {
        setLastError("Database not connected");
        return -1;
    }

    const char* sql =
        "INSERT INTO Trial (trial_id, config_hash, seed, time_limit, num_kits, num_modules, num_high_priority) "
        "VALUES (?, ?, ?, ?, ?, ?, ?)";

    sqlite3_stmt* stmt;
    int result = sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr);
    if (result != SQLITE_OK) {
        setLastError("Failed to prepare insert trial statement: " + std::string(sqlite3_errmsg(db_)));
        return -1;
    }

    bindParameter(stmt, 1, trial_data.trial_id);
    bindParameter(stmt, 2, trial_data.config_hash);
    bindParameter(stmt, 3, trial_data.seed);
    bindParameter(stmt, 4, trial_data.time_limit);
    bindParameter(stmt, 5, trial_data.num_kits);
    bindParameter(stmt, 6, trial_data.num_modules);
    bindParameter(stmt, 7, trial_data.num_high_priority);

    result = sqlite3_step(stmt);
    int trial_id = -1;

    if (result == SQLITE_DONE) {
        trial_id = static_cast<int>(sqlite3_last_insert_rowid(db_));
        ARIAC_DB_LOG("Inserted trial '" << trial_data.trial_id << "' with ID: " << trial_id);
    } else {
        setLastError("Failed to insert trial: " + std::string(sqlite3_errmsg(db_)));
    }

    sqlite3_finalize(stmt);
    return trial_id;
}

bool DatabaseManager::updateTrialConfigHash(const std::string& trial_id, const std::string& new_config_hash) {
    std::lock_guard<std::mutex> lock(db_mutex_);

    if (!connected_) {
        setLastError("Database not connected");
        return false;
    }

    const char* sql = "UPDATE Trial SET config_hash = ? WHERE trial_id = ?";

    sqlite3_stmt* stmt;
    int result = sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr);
    if (result != SQLITE_OK) {
        setLastError("Failed to prepare update trial statement: " + std::string(sqlite3_errmsg(db_)));
        return false;
    }

    bindParameter(stmt, 1, new_config_hash);
    bindParameter(stmt, 2, trial_id);

    result = sqlite3_step(stmt);
    bool success = (result == SQLITE_DONE);

    if (!success) {
        setLastError("Failed to update trial config hash: " + std::string(sqlite3_errmsg(db_)));
    } else {
        ARIAC_DB_LOG("Updated config hash for trial '" << trial_id << "'");
    }

    sqlite3_finalize(stmt);
    return success;
}

std::string DatabaseManager::hashFileSHA256(const std::string& file_path) {
    try {
        return hashFileSHA256Internal(file_path);
    } catch (const std::exception& e) {
        setLastError("Failed to hash file: " + std::string(e.what()));
        return "";
    }
}

int DatabaseManager::insertPenalty(int run_id, const PenaltyData& penalty_data) {
    // Check if run exists (this will handle its own locking)
    if (!runExists(run_id)) {
        setLastError("Run ID " + std::to_string(run_id) + " does not exist");
        return -1;
    }

    std::lock_guard<std::mutex> lock(db_mutex_);

    if (!connected_) {
        setLastError("Database not connected");
        return -1;
    }

    const char* sql = "INSERT INTO Penalty (run_id, type, description, time) VALUES (?, ?, ?, ?)";

    sqlite3_stmt* stmt;
    int result = sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr);
    if (result != SQLITE_OK) {
        setLastError("Failed to prepare insert penalty statement: " + std::string(sqlite3_errmsg(db_)));
        return -1;
    }

    bindParameter(stmt, 1, run_id);
    bindParameter(stmt, 2, penalty_data.type);
    bindParameter(stmt, 3, penalty_data.description);
    bindParameter(stmt, 4, penalty_data.time);

    result = sqlite3_step(stmt);
    int penalty_id = -1;

    if (result == SQLITE_DONE) {
        penalty_id = static_cast<int>(sqlite3_last_insert_rowid(db_));
        ARIAC_DB_LOG("Inserted penalty with ID: " << penalty_id << " for run ID: " << run_id);
    } else {
        setLastError("Failed to insert penalty: " + std::string(sqlite3_errmsg(db_)));
    }

    sqlite3_finalize(stmt);
    return penalty_id;
}

std::vector<PenaltyData> DatabaseManager::getPenaltiesForRun(int run_id) {
    std::lock_guard<std::mutex> lock(db_mutex_);

    std::vector<PenaltyData> penalties;

    if (!connected_) {
        setLastError("Database not connected");
        return penalties;
    }

    const char* sql = "SELECT id, run_id, type, description, time FROM Penalty WHERE run_id = ? ORDER BY time";

    sqlite3_stmt* stmt;
    int result = sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr);
    if (result != SQLITE_OK) {
        setLastError("Failed to prepare penalties query: " + std::string(sqlite3_errmsg(db_)));
        return penalties;
    }

    bindParameter(stmt, 1, run_id);

    while ((result = sqlite3_step(stmt)) == SQLITE_ROW) {
        PenaltyData penalty;
        penalty.id = getColumnInt(stmt, 0);
        penalty.run_id = getColumnInt(stmt, 1);
        penalty.type = getColumnInt(stmt, 2);
        penalty.description = getColumnText(stmt, 3);
        penalty.time = getColumnDouble(stmt, 4);
        penalties.push_back(penalty);
    }

    sqlite3_finalize(stmt);
    ARIAC_DB_LOG("Retrieved " << penalties.size() << " penalties for run ID: " << run_id);
    return penalties;
}

int DatabaseManager::insertOrderSubmission(int run_id, const OrderSubmissionData& order_data) {
    // Check if run exists (this will handle its own locking)
    if (!runExists(run_id)) {
        setLastError("Run ID " + std::to_string(run_id) + " does not exist");
        return -1;
    }

    std::lock_guard<std::mutex> lock(db_mutex_);

    if (!connected_) {
        setLastError("Database not connected");
        return -1;
    }

    const char* sql = 
        "INSERT INTO OrderSubmission (run_id, submission_time, announcement_time, time_limit, order_type) "
        "VALUES (?, ?, ?, ?, ?)";

    sqlite3_stmt* stmt;
    int result = sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr);
    if (result != SQLITE_OK) {
        setLastError("Failed to prepare insert order submission statement: " + std::string(sqlite3_errmsg(db_)));
        return -1;
    }

    bindParameter(stmt, 1, run_id);
    bindParameter(stmt, 2, order_data.submission_time);
    bindParameter(stmt, 3, order_data.announcement_time);
    bindParameter(stmt, 4, order_data.time_limit);
    
    // Handle order_type - if UNKNOWN, insert NULL
    if (order_data.order_type == OrderType::UNKNOWN) {
        sqlite3_bind_null(stmt, 5);
    } else {
        bindParameter(stmt, 5, orderTypeToInt(order_data.order_type));
    }

    result = sqlite3_step(stmt);
    int order_id = -1;

    if (result == SQLITE_DONE) {
        order_id = static_cast<int>(sqlite3_last_insert_rowid(db_));
        ARIAC_DB_LOG("Inserted order submission with ID: " << order_id << " for run ID: " << run_id 
                     << " (type: " << orderTypeToString(order_data.order_type) << ")");
    } else {
        setLastError("Failed to insert order submission: " + std::string(sqlite3_errmsg(db_)));
    }

    sqlite3_finalize(stmt);
    return order_id;
}

std::vector<OrderSubmissionData> DatabaseManager::getOrderSubmissionsForRun(int run_id) {
    std::lock_guard<std::mutex> lock(db_mutex_);

    std::vector<OrderSubmissionData> orders;

    if (!connected_) {
        setLastError("Database not connected");
        return orders;
    }

    const char* sql = 
        "SELECT id, run_id, submission_time, announcement_time, time_limit, order_type "
        "FROM OrderSubmission WHERE run_id = ? ORDER BY submission_time";

    sqlite3_stmt* stmt;
    int result = sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr);
    if (result != SQLITE_OK) {
        setLastError("Failed to prepare order submissions query: " + std::string(sqlite3_errmsg(db_)));
        return orders;
    }

    bindParameter(stmt, 1, run_id);

    while ((result = sqlite3_step(stmt)) == SQLITE_ROW) {
        OrderSubmissionData order;
        order.id = getColumnInt(stmt, 0);
        order.run_id = getColumnInt(stmt, 1);
        order.submission_time = getColumnDouble(stmt, 2);
        order.announcement_time = getColumnDouble(stmt, 3);
        order.time_limit = getColumnDouble(stmt, 4);
        
        // Handle order_type - check if NULL
        if (sqlite3_column_type(stmt, 5) == SQLITE_NULL) {
            order.order_type = OrderType::UNKNOWN;
        } else {
            order.order_type = intToOrderType(getColumnInt(stmt, 5));
        }
        
        orders.push_back(order);
    }

    sqlite3_finalize(stmt);
    ARIAC_DB_LOG("Retrieved " << orders.size() << " order submissions for run ID: " << run_id);
    return orders;
}

OrderSubmissionStats DatabaseManager::getOrderSubmissionStats(OrderType order_type) {
    std::lock_guard<std::mutex> lock(db_mutex_);

    OrderSubmissionStats stats;

    if (!connected_) {
        setLastError("Database not connected");
        return stats;
    }

    std::string sql = 
        "SELECT COUNT(*) as total, "
        "SUM(CASE WHEN submission_time <= time_limit THEN 1 ELSE 0 END) as on_time, "
        "AVG(submission_time - announcement_time) as avg_response_time "
        "FROM OrderSubmission "
        "WHERE announcement_time IS NOT NULL AND time_limit IS NOT NULL";

    // Add order type filter if specified
    if (order_type != OrderType::UNKNOWN) {
        sql += " AND order_type = " + std::to_string(orderTypeToInt(order_type));
    }

    sqlite3_stmt* stmt;
    int result = sqlite3_prepare_v2(db_, sql.c_str(), -1, &stmt, nullptr);
    if (result != SQLITE_OK) {
        setLastError("Failed to prepare order submission stats query: " + std::string(sqlite3_errmsg(db_)));
        return stats;
    }

    if (sqlite3_step(stmt) == SQLITE_ROW) {
        stats.total_submissions = getColumnInt(stmt, 0);
        stats.on_time_submissions = getColumnInt(stmt, 1);
        stats.avg_response_time = getColumnDouble(stmt, 2);
        
        if (stats.total_submissions > 0) {
            stats.success_rate = (static_cast<double>(stats.on_time_submissions) / stats.total_submissions) * 100.0;
        }
    }

    sqlite3_finalize(stmt);
    ARIAC_DB_LOG("Retrieved order submission stats for " << orderTypeToString(order_type) 
                 << ": " << stats.total_submissions << " total, " 
                 << stats.on_time_submissions << " on-time");
    return stats;
}

OrderSubmissionStats DatabaseManager::getOrderSubmissionStatsForRun(int run_id, OrderType order_type) {
    std::lock_guard<std::mutex> lock(db_mutex_);

    OrderSubmissionStats stats;

    if (!connected_) {
        setLastError("Database not connected");
        return stats;
    }

    std::string sql = 
        "SELECT COUNT(*) as total, "
        "SUM(CASE WHEN submission_time <= time_limit THEN 1 ELSE 0 END) as on_time, "
        "AVG(submission_time - announcement_time) as avg_response_time "
        "FROM OrderSubmission "
        "WHERE run_id = ? AND announcement_time IS NOT NULL AND time_limit IS NOT NULL";

    // Add order type filter if specified
    if (order_type != OrderType::UNKNOWN) {
        sql += " AND order_type = " + std::to_string(orderTypeToInt(order_type));
    }

    sqlite3_stmt* stmt;
    int result = sqlite3_prepare_v2(db_, sql.c_str(), -1, &stmt, nullptr);
    if (result != SQLITE_OK) {
        setLastError("Failed to prepare order submission stats for run query: " + std::string(sqlite3_errmsg(db_)));
        return stats;
    }

    bindParameter(stmt, 1, run_id);

    if (sqlite3_step(stmt) == SQLITE_ROW) {
        stats.total_submissions = getColumnInt(stmt, 0);
        stats.on_time_submissions = getColumnInt(stmt, 1);
        stats.avg_response_time = getColumnDouble(stmt, 2);
        
        if (stats.total_submissions > 0) {
            stats.success_rate = (static_cast<double>(stats.on_time_submissions) / stats.total_submissions) * 100.0;
        }
    }

    sqlite3_finalize(stmt);
    return stats;
}

bool DatabaseManager::executeQuery(const std::string& query) {
    std::lock_guard<std::mutex> lock(db_mutex_);
    return executeSql(query);
}

std::string DatabaseManager::getSchemaVersion() {
    // This is a placeholder - you might want to add a version table
    return "1.1.0";  // Updated version to reflect order_type support
}

bool DatabaseManager::validateSchema() {
    if (!connected_ || !db_)
        return false;

    // Check if required tables exist
    std::vector<std::string> required_tables = {"Competitor", "Trial", "Run", "Penalty", "OrderSubmission"};

    for (const auto& table : required_tables) {
        std::string sql = "SELECT name FROM sqlite_master WHERE type='table' AND name='" + table + "'";
        sqlite3_stmt* stmt;
        int result = sqlite3_prepare_v2(db_, sql.c_str(), -1, &stmt, nullptr);
        if (result != SQLITE_OK) {
            setLastError("Failed to prepare schema validation query: " + std::string(sqlite3_errmsg(db_)));
            sqlite3_finalize(stmt);
            return false;
        }

        bool table_exists = (sqlite3_step(stmt) == SQLITE_ROW);
        sqlite3_finalize(stmt);

        if (!table_exists) {
            setLastError("Required table '" + table + "' not found in database");
            ARIAC_DB_ERROR("Missing table: " + table);
            return false;
        } else {
            ARIAC_DB_LOG("Found required table: " + table);
        }
    }

    // Check if OrderSubmission table has order_type column (optional for backward compatibility)
    std::string check_order_type_sql = "PRAGMA table_info(OrderSubmission)";
    sqlite3_stmt* stmt;
    int result = sqlite3_prepare_v2(db_, check_order_type_sql.c_str(), -1, &stmt, nullptr);
    if (result == SQLITE_OK) {
        bool has_order_type = false;
        while (sqlite3_step(stmt) == SQLITE_ROW) {
            std::string column_name = getColumnText(stmt, 1);
            if (column_name == "order_type") {
                has_order_type = true;
                break;
            }
        }
        sqlite3_finalize(stmt);
        
        if (has_order_type) {
            ARIAC_DB_LOG("OrderSubmission table has order_type column");
        } else {
            ARIAC_DB_LOG("Warning: OrderSubmission table missing order_type column (backward compatibility mode)");
        }
    }

    ARIAC_DB_LOG("Database schema validation passed");
    return true;
}

bool DatabaseManager::beginTransaction() {
    return executeSql("BEGIN TRANSACTION");
}

bool DatabaseManager::commitTransaction() {
    return executeSql("COMMIT");
}

bool DatabaseManager::rollbackTransaction() {
    return executeSql("ROLLBACK");
}

// Private helper methods

bool DatabaseManager::executeSql(const std::string& sql) {
    if (!connected_) {
        setLastError("Database not connected");
        return false;
    }

    char* error_msg = nullptr;
    int result = sqlite3_exec(db_, sql.c_str(), nullptr, nullptr, &error_msg);

    if (result != SQLITE_OK) {
        std::string error = "SQL execution failed: ";
        if (error_msg) {
            error += error_msg;
            sqlite3_free(error_msg);
        }
        setLastError(error);
        return false;
    }

    return true;
}

void DatabaseManager::setLastError(const std::string& error) {
    last_error_ = error;
    ARIAC_DB_ERROR(error);
}

void DatabaseManager::bindParameter(sqlite3_stmt* stmt, int index, const std::string& value) {
    sqlite3_bind_text(stmt, index, value.c_str(), -1, SQLITE_STATIC);
}

void DatabaseManager::bindParameter(sqlite3_stmt* stmt, int index, int value) {
    sqlite3_bind_int(stmt, index, value);
}

void DatabaseManager::bindParameter(sqlite3_stmt* stmt, int index, double value) {
    sqlite3_bind_double(stmt, index, value);
}

void DatabaseManager::bindParameter(sqlite3_stmt* stmt, int index, bool value) {
    sqlite3_bind_int(stmt, index, value ? 1 : 0);
}

std::string DatabaseManager::getColumnText(sqlite3_stmt* stmt, int index) {
    const char* text = reinterpret_cast<const char*>(sqlite3_column_text(stmt, index));
    return text ? std::string(text) : std::string();
}

int DatabaseManager::getColumnInt(sqlite3_stmt* stmt, int index) {
    return sqlite3_column_int(stmt, index);
}

double DatabaseManager::getColumnDouble(sqlite3_stmt* stmt, int index) {
    return sqlite3_column_double(stmt, index);
}

bool DatabaseManager::getColumnBool(sqlite3_stmt* stmt, int index) {
    return sqlite3_column_int(stmt, index) != 0;
}

std::string DatabaseManager::hashFileSHA256Internal(const std::string& filePath) {
    std::ifstream file(filePath, std::ios::binary);
    if (!file)
        throw std::runtime_error("Could not open file: " + filePath);

    EVP_MD_CTX* ctx = EVP_MD_CTX_new();
    if (!ctx)
        throw std::runtime_error("Failed to create EVP_MD_CTX");

    if (EVP_DigestInit_ex(ctx, EVP_sha256(), nullptr) != 1)
        throw std::runtime_error("EVP_DigestInit_ex failed");

    char buffer[4096];
    while (file.read(buffer, sizeof(buffer)))
        EVP_DigestUpdate(ctx, buffer, file.gcount());
    if (file.gcount() > 0)
        EVP_DigestUpdate(ctx, buffer, file.gcount());

    unsigned char hash[EVP_MAX_MD_SIZE];
    unsigned int lengthOfHash = 0;

    if (EVP_DigestFinal_ex(ctx, hash, &lengthOfHash) != 1)
        throw std::runtime_error("EVP_DigestFinal_ex failed");

    EVP_MD_CTX_free(ctx);

    std::ostringstream result;
    for (unsigned int i = 0; i < lengthOfHash; ++i)
        result << std::hex << std::setw(2) << std::setfill('0') << (int)hash[i];

    return result.str();
}

}  // namespace ariac_db