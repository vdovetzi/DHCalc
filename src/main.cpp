#include "dhcalc/dh_table_parser.hpp"
#include "dhcalc/kinematics.hpp"

#include <argparse/argparse.hpp>

#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

int main(int argc, char *argv[]) {
  argparse::ArgumentParser program("dhcalc", "0.0.1");

  program.add_description(
      "Compute the base-to-end-effector homogeneous transformation matrix from "
      "a Denavit-Hartenberg TSV table.\n");

  program.add_argument("-i", "--input")
      .help("path to DH table (.tsv file)")
      .metavar("DH_TABLE");

  program.add_argument("-p", "--precision")
      .help("number of decimal digits in the output (0–15)")
      .default_value(6)
      .scan<'i', int>()
      .metavar("N");

  // ── Parse arguments ─────────────────────────────────────────────────────
  try {
    program.parse_args(argc, argv);
  } catch (const std::exception &err) {
    std::cerr << err.what() << "\n\n";
    std::cerr << program;
    return EXIT_FAILURE;
  }

  // ── Validate precision ──────────────────────────────────────────────────
  const int precision = program.get<int>("--precision");
  if (precision < 0 || precision > 15) {
    std::cerr << "dhcalc error: --precision must be between 0 and 15.\n";
    return EXIT_FAILURE;
  }

  // ── Run FK pipeline ─────────────────────────────────────────────────────
  const std::filesystem::path input_path = program.get<std::string>("input");

  try {
    const std::vector<dhcalc::DHFrameParameters> joints =
        dhcalc::parse_dh_table(input_path);

    const dhcalc::Matrix4 transform = dhcalc::compute_FK(joints);

    std::cout << "Base-to-end-effector homogeneous transformation matrix\n"
              << dhcalc::format_matrix(transform, precision) << '\n';

    return EXIT_SUCCESS;
  } catch (const dhcalc::ParseError &err) {
    std::cerr << "dhcalc parse error: " << err.what() << '\n';
    return EXIT_FAILURE;
  } catch (const std::exception &err) {
    std::cerr << "dhcalc error: " << err.what() << '\n';
    return EXIT_FAILURE;
  }
}
