/**
 * @file ariac_database.hpp
 * @brief ARIAC 2025 Database Interface
 * @author ARIAC Development Team
 * @version 1.0.0
 *
 * C++ interface for the ARIAC 2025 SQLite database.
 * Provides type-safe access to competition data including trials, competitors, and runs.
 */

#pragma once

#include <sqlite3.h>

#include <functional>  // Required for std::function
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <fstream>
#include <iomanip>
#include <sstream>

#include <openssl/evp.h>

#ifdef ARIAC_DB_DEBUG
#define ARIAC_DB_LOG(msg) std::cout << "[ARIAC_DB] " << msg << std::endl
#define ARIAC_DB_ERROR(msg) std::cerr << "[ARIAC_DB ERROR] " << msg << std::endl
#else
#define ARIAC_DB_LOG(msg)
#define ARIAC_DB_ERROR(msg) std::cerr << "[ARIAC_DB ERROR] " << msg << std::endl
#endif

namespace ariac_db {

/**
 * @brief Enumeration for order types
 */
enum class OrderType {
    KIT = 1,
    MODULE = 2,
    HIGH_PRIORITY = 3,
    UNKNOWN = -1
};

/**
 * @brief Convert OrderType enum to string
 */
std::string orderTypeToString(OrderType type);

/**
 * @brief Convert integer to OrderType enum
 */
OrderType intToOrderType(int type);

/**
 * @brief Convert OrderType enum to integer
 */
int orderTypeToInt(OrderType type);

/**
 * @brief Data structure representing a trial run
 */
struct RunData {
    int id = -1;                    ///< Run ID (auto-generated)
    int trial_id = -1;              ///< Foreign key to Trial table
    int competitor_id = -1;         ///< Foreign key to Competitor table
    bool completed = false;         ///< Whether run was completed
    bool aborted = false;           ///< Whether run was aborted
    double sensor_cost = 0.0;       ///< Cost of sensors used
    double duration = 0.0;          ///< Duration of the run
    int total_cells = 0;            ///< Total number of cells
    int defective_cells = 0;        ///< Number of defective cells
    double avg_report_time = 0.0;   ///< Average report time
    int num_reports_submitted = 0;  ///< Number of reports submitted
    int num_correct_reports = 0;    ///< Number of correct reports
    int num_correct_report_classifications = 0; ///< Number of correct report classifications
};

/**
 * @brief Data structure representing a competitor/team
 */
struct CompetitorData {
    int id = -1;       ///< Competitor ID (auto-generated)
    std::string name;  ///< Team name
};

/**
 * @brief Data structure representing a trial
 */
struct TrialData {
    int id = -1;              ///< Trial ID (auto-generated)
    std::string trial_id;     ///< Trial identifier string
    std::string config_hash;  ///< Configuration hash
    int seed = 0;             ///< Random seed
    int time_limit = 0;       ///< Time limit in seconds
    int num_kits = 0;         ///< Total number of kits
    int num_modules = 0;      ///< Total number of modules
    int num_high_priority = 0;      ///< Total number of high priority orders
};

/**
 * @brief Data structure representing a penalty
 */
struct PenaltyData {
    int id = -1;              ///< Penalty ID (auto-generated)
    int run_id = -1;          ///< Foreign key to Run table
    int type = 0;             ///< Penalty type
    std::string description;  ///< Penalty description
    double time = 0.0;        ///< Time when penalty occurred
};

/**
 * @brief Data structure representing an order submission
 */
struct OrderSubmissionData {
    int id = -1;                    ///< Order submission ID (auto-generated)
    int run_id = -1;                ///< Foreign key to Run table
    double submission_time = 0.0;   ///< Submission time
    double announcement_time = 0.0; ///< Announcement time
    double time_limit = 0.0;        ///< Time limit
    OrderType order_type = OrderType::UNKNOWN; ///< Order type (Assembly, Kitting, Combined)
    
    /**
     * @brief Calculate response time (submission_time - announcement_time)
     */
    double getResponseTime() const {
        return submission_time - announcement_time;
    }
    
    /**
     * @brief Check if submission was on time
     */
    bool isOnTime() const {
        return submission_time <= time_limit;
    }
};

/**
 * @brief Structure for order submission statistics
 */
struct OrderSubmissionStats {
    int total_submissions = 0;
    int on_time_submissions = 0;
    double avg_response_time = 0.0;
    double success_rate = 0.0;  ///< Percentage of on-time submissions
};

/**
 * @brief Main database interface class
 *
 * Provides type-safe, thread-safe access to the ARIAC 2025 SQLite database.
 * Supports reading and writing competition data including trials, competitors, and runs.
 */
class DatabaseManager {
   public:
    /**
     * @brief Constructor
     * @param db_path Path to the SQLite database file
     */
    explicit DatabaseManager(const std::string& db_path);

    /**
     * @brief Destructor - automatically closes database connection
     */
    ~DatabaseManager();

    // Disable copy constructor and assignment operator
    DatabaseManager(const DatabaseManager&) = delete;
    DatabaseManager& operator=(const DatabaseManager&) = delete;

    // Enable move constructor and assignment operator
    DatabaseManager(DatabaseManager&& other) noexcept;
    DatabaseManager& operator=(DatabaseManager&& other) noexcept;

    /**
     * @brief Connect to the database
     * @return True if connection successful, false otherwise
     */
    bool connect();

    /**
     * @brief Disconnect from the database
     */
    void disconnect();

    /**
     * @brief Check if database is connected
     * @return True if connected, false otherwise
     */
    bool isConnected() const;

    /**
     * @brief Get last error message
     * @return Error message string
     */
    std::string getLastError() const;

    // === Run Operations ===

    /**
     * @brief Insert a new run into the database
     * @param trial_id Trial ID (from Trial table)
     * @param competitor_id Competitor ID (from Competitor table)
     * @param data Run data to insert
     * @return Run ID if successful, -1 if failed
     */
    int insertRun(int trial_id, int competitor_id, const RunData& data);

    /**
     * @brief Insert a new run for a specific trial (by trial_id string)
     * @param trial_id_str Trial identifier string
     * @param competitor_id Competitor ID
     * @param data Run data to insert
     * @return Run ID if successful, -1 if failed
     */
    int insertRunForTrial(const std::string& trial_id_str, int competitor_id, const RunData& data);

    /**
     * @brief Update an existing run in the database
     * @param run_id Run ID to update
     * @param data New run data
     * @return True if successful, false otherwise
     */
    bool updateRun(int run_id, const RunData& data);

    /**
     * @brief Get runs with optional WHERE clause
     * @param where_clause SQL WHERE clause (without "WHERE" keyword)
     * @return Vector of RunData structures
     */
    std::vector<RunData> getRuns(const std::string& where_clause = "");

    /**
     * @brief Get a specific run by ID
     * @param run_id Run ID to retrieve
     * @return RunData structure (id will be -1 if not found)
     */
    RunData getRun(int run_id);

    /**
     * @brief Get all runs for a specific team
     * @param team_name Team name
     * @return Vector of RunData structures
     */
    std::vector<RunData> getRunsByTeam(const std::string& team_name);

    /**
     * @brief Get all runs for a specific trial
     * @param trial_id Trial identifier string
     * @return Vector of RunData structures
     */
    std::vector<RunData> getRunsByTrial(const std::string& trial_id);

    /**
     * @brief Check if a run exists
     * @param run_id Run ID to check
     * @return True if run exists, false otherwise
     */
    bool runExists(int run_id);

    // === Competitor Operations ===

    /**
     * @brief Get competitor ID by team name
     * @param team_name Team name
     * @return Competitor ID if found, -1 if not found
     */
    int getCompetitorId(const std::string& team_name);

    /**
     * @brief Get competitor data by team name
     * @param team_name Team name
     * @return CompetitorData structure (id will be -1 if not found)
     */
    CompetitorData getCompetitor(const std::string& team_name);

    /**
     * @brief Get all competitors
     * @return Vector of CompetitorData structures
     */
    std::vector<CompetitorData> getAllCompetitors();

    /**
     * @brief Insert a new competitor
     * @param team_name Team name
     * @return Competitor ID if successful, -1 if failed
     */
    int insertCompetitor(const std::string& team_name);

    // === Trial Operations ===

    /**
     * @brief Get trial ID (primary key) by trial identifier string
     * @param trial_id Trial identifier string
     * @return Trial ID (primary key) if found, -1 if not found
     */
    int getTrialId(const std::string& trial_id);

    /**
     * @brief Get trial data by trial identifier string
     * @param trial_id Trial identifier string
     * @return TrialData structure (id will be -1 if not found)
     */
    TrialData getTrial(const std::string& trial_id);

    /**
     * @brief Get all trials
     * @return Vector of TrialData structures
     */
    std::vector<TrialData> getAllTrials();

    /**
     * @brief Insert a new trial
     * @param trial_data Trial data to insert
     * @return Trial ID if successful, -1 if failed
     */
    int insertTrial(const TrialData& trial_data);

    /**
     * @brief Update an existing trial's configuration hash
     * @param trial_id Trial identifier string
     * @param new_config_hash New configuration hash
     * @return True if successful, false otherwise
     */
    bool updateTrialConfigHash(const std::string& trial_id, const std::string& new_config_hash);

    /**
     * @brief Generate SHA256 hash for a file
     * @param file_path Path to the file to hash
     * @return SHA256 hash as hex string, empty string on error
     */
    std::string hashFileSHA256(const std::string& file_path);

    // === Penalty Operations ===

    /**
     * @brief Insert a penalty for a specific run
     * @param run_id Run ID
     * @param penalty_data Penalty data to insert
     * @return Penalty ID if successful, -1 if failed
     */
    int insertPenalty(int run_id, const PenaltyData& penalty_data);

    /**
     * @brief Get all penalties for a specific run
     * @param run_id Run ID
     * @return Vector of PenaltyData structures
     */
    std::vector<PenaltyData> getPenaltiesForRun(int run_id);

    // === Order Submission Operations ===

    /**
     * @brief Insert an order submission for a specific run
     * @param run_id Run ID
     * @param order_data Order submission data to insert
     * @return Order submission ID if successful, -1 if failed
     */
    int insertOrderSubmission(int run_id, const OrderSubmissionData& order_data);

    /**
     * @brief Get all order submissions for a specific run
     * @param run_id Run ID
     * @return Vector of OrderSubmissionData structures
     */
    std::vector<OrderSubmissionData> getOrderSubmissionsForRun(int run_id);

    /**
     * @brief Get order submission statistics by order type
     * @param order_type Specific order type (optional, use OrderType::UNKNOWN for all types)
     * @return OrderSubmissionStats structure
     */
    OrderSubmissionStats getOrderSubmissionStats(OrderType order_type = OrderType::UNKNOWN);

    /**
     * @brief Get order submission statistics by order type for a specific run
     * @param run_id Run ID
     * @param order_type Specific order type (optional, use OrderType::UNKNOWN for all types)
     * @return OrderSubmissionStats structure
     */
    OrderSubmissionStats getOrderSubmissionStatsForRun(int run_id, OrderType order_type = OrderType::UNKNOWN);

    // === Utility Operations ===

    /**
     * @brief Execute a custom SQL query (read-only)
     * @param query SQL query string
     * @return True if successful, false otherwise
     */
    bool executeQuery(const std::string& query);

    /**
     * @brief Get database schema version
     * @return Schema version string
     */
    std::string getSchemaVersion();

    /**
     * @brief Check if database tables exist
     * @return True if all required tables exist, false otherwise
     */
    bool validateSchema();

    /**
     * @brief Begin a database transaction
     * @return True if successful, false otherwise
     */
    bool beginTransaction();

    /**
     * @brief Commit a database transaction
     * @return True if successful, false otherwise
     */
    bool commitTransaction();

    /**
     * @brief Rollback a database transaction
     * @return True if successful, false otherwise
     */
    bool rollbackTransaction();

   private:
    std::string db_path_;          ///< Database file path
    sqlite3* db_;                  ///< SQLite database handle
    bool connected_;               ///< Connection status
    mutable std::mutex db_mutex_;  ///< Mutex for thread safety
    std::string last_error_;       ///< Last error message

    /**
     * @brief Execute a SQL statement
     * @param sql SQL statement
     * @return True if successful, false otherwise
     */
    bool executeSql(const std::string& sql);

    /**
     * @brief Set the last error message
     * @param error Error message
     */
    void setLastError(const std::string& error);

    /**
     * @brief Bind parameters to a prepared statement
     * @param stmt Prepared statement
     * @param index Parameter index (1-based)
     * @param value Value to bind
     */
    void bindParameter(sqlite3_stmt* stmt, int index, const std::string& value);
    void bindParameter(sqlite3_stmt* stmt, int index, int value);
    void bindParameter(sqlite3_stmt* stmt, int index, double value);
    void bindParameter(sqlite3_stmt* stmt, int index, bool value);

    /**
     * @brief Get column value from result set
     * @param stmt Prepared statement
     * @param index Column index (0-based)
     * @return Column value as string/int/double/bool
     */
    std::string getColumnText(sqlite3_stmt* stmt, int index);
    int getColumnInt(sqlite3_stmt* stmt, int index);
    double getColumnDouble(sqlite3_stmt* stmt, int index);
    bool getColumnBool(sqlite3_stmt* stmt, int index);

    /**
     * @brief Internal implementation of SHA256 hashing
     * @param filePath Path to file to hash
     * @return SHA256 hash as hex string
     */
    std::string hashFileSHA256Internal(const std::string& filePath);
};

/**
 * @brief RAII wrapper for database transactions
 */
class DatabaseTransaction {
   public:
    explicit DatabaseTransaction(DatabaseManager& db) : db_(db), committed_(false) {
        success_ = db_.beginTransaction();
    }

    ~DatabaseTransaction() {
        if (success_ && !committed_) {
            db_.rollbackTransaction();
        }
    }

    bool commit() {
        if (success_ && !committed_) {
            committed_ = true;
            return db_.commitTransaction();
        }
        return false;
    }

    bool isValid() const {
        return success_;
    }

   private:
    DatabaseManager& db_;
    bool success_;
    bool committed_;
};

}  // namespace ariac_db