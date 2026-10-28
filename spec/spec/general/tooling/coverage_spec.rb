# frozen_string_literal: true

require "stringio"
require "tmpdir"

# Firmware code coverage (`--coverage`): lcov mid-run over the control API and at exit, and
# merging tracefiles (TrmnlSim::Lcov).

General.describe "Coverage" do
  describe "Coverage" do
    around do |example|
      Dir.mktmpdir("trmnl-cov-") do |dir|
        @dir = dir
        example.run
      end
    end

    def load(name) = TrmnlSim::Lcov::Report.load([File.join(@dir, name)])

    it "setup boot covers bl_init but not error paths" do
      final = File.join(@dir, "final.info")
      again = sim(erase: true, coverage: final, extra_args: ["--offline"]) do |s|
        s.wait(portal: true, timeout: 90)
        mid = s.write_coverage(File.join(@dir, "mid.info"))
        expect(mid["lines_hit"]).to be > 1000
        expect(mid["lines_hit"]).to be < mid["lines_found"]

        cov = load("mid.info")
        # Paths of the firmware's own sources are relative to its checkout.
        bl = cov["src/bl.cpp"]
        main = cov["src/main.cpp"]
        expect(bl.functions["bl_init()"][1]).to be > 0
        expect(main.functions["setup()"][1]).to be > 0
        first_line = bl.functions["bl_init()"][0]
        expect(bl.lines[first_line]).to be > 0
        # Only reached when joining WiFi fails.
        sleep = cov["src/sleep_session.cpp"]
        expect(sleep.functions["wifiErrorDeepSleep()"][1]).to eq(0)
        line = sleep.functions["wifiErrorDeepSleep()"][0]
        expect(sleep.lines[line]).to eq(0)

        # reset: start over; the portal loop keeps running, the boot code doesn't.
        s.write_coverage(File.join(@dir, "before-reset.info"), reset: true)
        again = s.write_coverage(File.join(@dir, "after-reset.info"))
        expect(again["lines_hit"]).to be < mid["lines_hit"] / 2
        expect(load("after-reset.info")["src/bl.cpp"].lines[first_line]).to eq(0)
        again
      end

      # Written on exit too: everything since the reset.
      expect(File).to exist(final)
      expect(TrmnlSim::Lcov::Report.load([final]).totals.lines_hit).to be >= again["lines_hit"]
    end

    it "merge tool unions runs" do
      File.write(File.join(@dir, "a.info"), "TN:\nSF:src/x.cpp\nFN:1,f()\nFNDA:1,f()\nDA:1,1\nDA:2,0\nend_of_record\n")
      File.write(File.join(@dir, "b.info"), "TN:\nSF:src/x.cpp\nFN:1,f()\nFNDA:0,f()\nDA:1,1\nDA:2,1\nDA:3,0\n" \
                                            "end_of_record\nSF:/idf/y.c\nDA:5,1\nend_of_record\n")
      out = File.join(@dir, "merged.info")
      printed = StringIO.new
      status = TrmnlSim::Lcov.main([@dir, "-o", out, "--include", "src/", "-q", "--html", File.join(@dir, "html")],
                                   out: printed)
      expect(status).to eq(0)
      expect(printed.string).to eq("lines 2/3 66.7%, functions 1/1 100.0% in 1 files (2 tracefiles)\n")
      cov = TrmnlSim::Lcov::Report.load([out]).files
      expect(cov.keys).to eq(["src/x.cpp"])
      expect(cov["src/x.cpp"].lines).to eq({ 1 => 2, 2 => 1, 3 => 0 })
      expect(cov["src/x.cpp"].functions).to eq({ "f()" => [1, 1] })
      expect(File).to exist(File.join(@dir, "html", "index.html"))
    end
  end
end
