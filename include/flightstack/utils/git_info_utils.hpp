/***********************************************************************************************************************
 * Copyright (c) 2024 Mattia Gramuglia, Giri M. Kumar, Andrea L'Afflitto. All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
 * following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
 *    disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 *    following disclaimer in the documentation and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * File:        git_info_utils.hpp
 * Author:      Mattia Gramuglia
 * Date:        June 19, 2025
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Functions to log the Git/GitHub info of when the code was run.
 * 
 * GitHub:    https://github.com/andrealaffly/ACSL-flightstack.git
 **********************************************************************************************************************/

#pragma once

#include <string>
#include <nlohmann/json.hpp>
#include <filesystem>
#include <array>
#include <cstdio>
#include <memory>
#include <iostream>

namespace acsl_utils {
namespace git_info {

struct GitRepoInfo {
  std::string repo_path;
  std::string remote_url;
  std::string commit_hash;
  std::string commit_tag;
  std::string branch;
  int dirty;
  std::string commit_url;

  nlohmann::json to_json() const {
    return {
      {"repo_path", repo_path},
      {"remote_url", remote_url},
      {"commit_hash", commit_hash},
      {"commit_tag", commit_tag},
      {"branch", branch},
      {"dirty", dirty},
      {"commit_url", commit_url}
    };
  }
};

inline std::string runGitCommand(const std::string& command, const std::string& cwd) {
  std::array<char, 128> buffer;
  std::string result;

  #if defined(_WIN32)
    std::string full_command = "cd /d \"" + cwd + "\" && " + command + " 2>nul";
  #else
    std::string full_command = "cd \"" + cwd + "\" && " + command + " 2>/dev/null";
  #endif

  std::shared_ptr<FILE> pipe(popen(full_command.c_str(), "r"), pclose);
  if (!pipe) return "";

  while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
    result += buffer.data();
  }

  if (!result.empty() && result.back() == '\n')
    result.pop_back();

  return result;
}

inline std::string findGitRoot(const std::filesystem::path& start_path) {
  auto path = start_path;
  while (!std::filesystem::exists(path / ".git")) {
    if (path.has_parent_path()) {
      path = path.parent_path();
    } else {
      return "";
    }
  }
  return path.string();
}

inline GitRepoInfo getGitRepoInfo() {
  // Use the location of the header/source file as the starting point
  std::string source_path = __FILE__;
  std::filesystem::path start_path(source_path);
  std::string repo_dir = findGitRoot(start_path);

  if (repo_dir.empty()) {
    std::cerr << "⚠️ Git repository info cannot be tracked. No .git directory found in hierarchy.\n";
    return {};
  }

  auto runGitCommand = [&repo_dir](const std::string& cmd) -> std::string {
    std::array<char, 256> buffer;
    std::string result;
    std::string full_cmd = "cd " + repo_dir + " && " + cmd + " 2>/dev/null";
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(full_cmd.c_str(), "r"), pclose);
    if (!pipe) return "";
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr)
      result += buffer.data();
    result.erase(std::remove(result.begin(), result.end(), '\n'), result.end());
    return result;
  };

  std::string remote_url = runGitCommand("git remote get-url origin");
  std::string commit_hash = runGitCommand("git rev-parse HEAD");
  if (remote_url.size() >= 4 && remote_url.substr(remote_url.size() - 4) == ".git") {
    remote_url.erase(remote_url.size() - 4);
  }

  return GitRepoInfo{
    repo_dir,
    remote_url,
    commit_hash,
    runGitCommand("git describe --tags --exact-match"),
    runGitCommand("git rev-parse --abbrev-ref HEAD"),
    !runGitCommand("git status --porcelain").empty(),
    remote_url + "/tree/" + commit_hash
  };
}


} // namespace git_info
} // namespace acsl_utils