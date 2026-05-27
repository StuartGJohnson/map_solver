#include "map_solver/scan_match_batch.hpp"

#include <algorithm>
#include <atomic>
#include <mutex>
#include <thread>
#include <utility>

namespace map_solver
{

std::vector<WaypointScan> loadWaypointScans(
  const std::filesystem::path & mdc_directory,
  const ScanBagConversionOptions & conversion_options)
{
  std::vector<std::filesystem::path> waypoint_dirs;
  if (!std::filesystem::exists(mdc_directory)) {
    return {};
  }

  for (const auto & entry : std::filesystem::directory_iterator(mdc_directory)) {
    if (!entry.is_directory()) {
      continue;
    }
    const auto name = entry.path().filename().string();
    if (name.rfind("waypoint_", 0) == 0) {
      waypoint_dirs.push_back(entry.path());
    }
  }
  std::sort(waypoint_dirs.begin(), waypoint_dirs.end());

  ScanBagConverter converter(conversion_options);
  std::vector<WaypointScan> scans;
  scans.reserve(waypoint_dirs.size());
  for (std::size_t i = 0; i < waypoint_dirs.size(); ++i) {
    scans.push_back(WaypointScan{i, converter.readWaypointDirectory(waypoint_dirs[i])});
  }
  return scans;
}

std::vector<ScanMatchJob> makeSequentialScanMatchJobs(
  const std::vector<WaypointScan> & waypoint_scans)
{
  if (waypoint_scans.size() < 2U) {
    return {};
  }

  std::vector<std::shared_ptr<const ScanBundle>> bundles;
  bundles.reserve(waypoint_scans.size());
  for (const auto & waypoint_scan : waypoint_scans) {
    bundles.push_back(std::make_shared<ScanBundle>(waypoint_scan.bundle));
  }

  std::vector<ScanMatchJob> jobs;
  jobs.reserve(waypoint_scans.size() - 1U);
  for (std::size_t i = 0; i + 1U < waypoint_scans.size(); ++i) {
    jobs.push_back(ScanMatchJob{
      waypoint_scans[i].index,
      waypoint_scans[i + 1U].index,
      bundles[i],
      bundles[i + 1U]});
  }
  return jobs;
}

std::vector<ScanMatchJobResult> runScanMatchJobsParallel(
  const std::vector<ScanMatchJob> & jobs,
  const ScanMatcherOptions & matcher_options,
  std::size_t thread_count)
{
  if (jobs.empty()) {
    return {};
  }

  const std::size_t hardware_threads = std::max(1U, std::thread::hardware_concurrency());
  const std::size_t worker_count = std::max<std::size_t>(
    1U, std::min(thread_count == 0U ? hardware_threads : thread_count, jobs.size()));

  std::atomic<std::size_t> next_job{0U};
  std::mutex results_mutex;
  std::vector<ScanMatchJobResult> successful_results;
  successful_results.reserve(jobs.size());

  auto worker = [&]() {
      ScanMatcher matcher(matcher_options);
      while (true) {
        const std::size_t job_index = next_job.fetch_add(1U);
        if (job_index >= jobs.size()) {
          break;
        }

        const auto & job = jobs[job_index];
        if (!job.target || !job.source) {
          continue;
        }

        auto match = matcher.match(job.target->points, job.source->points);
        if (!match.success) {
          continue;
        }

        ScanMatchJobResult job_result;
        job_result.target_index = job.target_index;
        job_result.source_index = job.source_index;
        job_result.target_bag_path = job.target->bag_path;
        job_result.source_bag_path = job.source->bag_path;
        job_result.match = std::move(match);

        std::lock_guard<std::mutex> lock(results_mutex);
        successful_results.push_back(std::move(job_result));
      }
    };

  std::vector<std::thread> workers;
  workers.reserve(worker_count);
  for (std::size_t i = 0; i < worker_count; ++i) {
    workers.emplace_back(worker);
  }
  for (auto & thread : workers) {
    thread.join();
  }

  std::sort(
    successful_results.begin(), successful_results.end(),
    [](const auto & lhs, const auto & rhs) {
      if (lhs.target_index != rhs.target_index) {
        return lhs.target_index < rhs.target_index;
      }
      return lhs.source_index < rhs.source_index;
    });
  return successful_results;
}

}  // namespace map_solver
