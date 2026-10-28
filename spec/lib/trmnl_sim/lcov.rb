# frozen_string_literal: true

require "fileutils"
require "optparse"
require "pathname"

module TrmnlSim
  # Merge lcov tracefiles from simulator runs (`trmnl-sim --coverage`) and report on them.
  #
  #     report = TrmnlSim::Lcov::Report.load(["cov/"], include: %w[src/ lib/])
  #     puts report.summary
  #     report.write_lcov("all.info")
  #     report.write_html("cov-html", root: "..")
  #
  # Inputs are tracefiles or directories of them. Line and function hit counts are summed (a
  # simulator records 0 or 1 per line, so a merged count is the number of runs that executed
  # it). `include:` keeps only files whose path starts with a prefix; paths are relative to the
  # firmware checkout for its own sources (see `--coverage-root`).
  #
  # The summary lists every file with its line and function coverage, then the total.
  # `write_html` writes a self-contained report (an index plus one page per file with the
  # source colored by coverage; sources are read from `root`). `genhtml` from lcov also reads
  # the merged tracefile, if it is installed.
  module Lcov
    # What the integration tests name their merge of a coverage directory.
    MERGED = "merged.info"

    # A source file's coverage: {line => hits} and {function name => [line, hits]}.
    FileCoverage = Struct.new(:lines, :functions) do
      def initialize(lines = {}, functions = {}) = super

      def lines_found = lines.size
      def lines_hit = lines.count { |_, n| n.positive? }
      def functions_found = functions.size
      def functions_hit = functions.count { |_, (_, n)| n.positive? }
    end

    Totals = Struct.new(:lines_hit, :lines_found, :functions_hit, :functions_found)

    module_function

    # The tracefiles of `inputs`: files as given; directories' *.info files, also in
    # subdirectories (one per build when the test support collects them), but not an earlier
    # merge of them (MERGED) or `exclude`.
    def tracefiles(inputs, exclude: nil)
      inputs.flat_map do |input|
        found = if File.directory?(input)
                  # (in path order, component by component: a/x before a-b/x)
                  Dir.glob("**/*.info", base: input).sort_by { _1.split("/") }.map { File.join(input, _1) }
                     .reject { File.basename(_1) == MERGED }
                else
                  [input]
                end
        found.reject { |f| exclude && File.expand_path(f) == File.expand_path(exclude) }
      end
    end

    # A file's text as UTF-8, invalid bytes replaced.
    def read_text(path) = File.binread(path).force_encoding(Encoding::UTF_8).scrub("\uFFFD")

    # "12.3%" (right-aligned in 5), or "    -" without anything to cover.
    def pct(hit, found) = found.positive? ? format("%5.1f%%", 100.0 * hit / found) : "    -"

    # Merged coverage: {source path => FileCoverage}, from `tracefiles` tracefiles.
    class Report
      attr_reader :files, :tracefiles

      # The merge of every tracefile of `inputs` (see Lcov.tracefiles).
      def self.load(inputs, include: [], exclude: nil)
        report = new(include:)
        Lcov.tracefiles(Array(inputs), exclude:).each { report.add(_1) }
        report
      end

      def initialize(include: [])
        @include = include
        @files = {}
        @tracefiles = 0
      end

      def [](path) = files[path]

      # Merge a tracefile in.
      def add(path)
        @tracefiles += 1
        cur = nil
        Lcov.read_text(path).each_line(chomp: true) do |line|
          key, _, val = line.partition(":")
          if key == "SF"
            cur = @include.empty? || val.start_with?(*@include) ? (files[val] ||= FileCoverage.new) : nil
          elsif cur.nil?
            next
          elsif key == "DA"
            ln, count = val.split(",").first(2).map { Integer(_1) }
            cur.lines[ln] = cur.lines.fetch(ln, 0) + count
          elsif key == "FN"
            # FN:<line>,<name> (lcov 2 may add an end line: FN:<line>,<end>,<name>)
            parts = val.split(",", 3)
            name = parts.size == 3 && parts[1].match?(/\A\d+\z/) ? parts[2] : val.split(",", 2)[1]
            cur.functions[name] ||= [Integer(parts[0]), 0]
          elsif key == "FNDA"
            count, name = val.split(",", 2)
            (cur.functions[name] ||= [0, 0])[1] += Integer(count)
          elsif key == "end_of_record"
            cur = nil
          end
        end
        self
      end

      def totals
        Totals.new(files.values.sum(&:lines_hit), files.values.sum(&:lines_found),
                   files.values.sum(&:functions_hit), files.values.sum(&:functions_found))
      end

      # "lines 1/2 50.0%, functions 1/1 100.0% in 1 files (2 tracefiles)"
      def total_line
        t = totals
        "lines #{t.lines_hit}/#{t.lines_found} #{Lcov.pct(t.lines_hit, t.lines_found).strip}, " \
          "functions #{t.functions_hit}/#{t.functions_found} #{Lcov.pct(t.functions_hit, t.functions_found).strip} " \
          "in #{files.size} files (#{tracefiles} tracefiles)"
      end

      # A table of every file's line and function coverage, then the total.
      def summary
        width = [*files.keys.map(&:size), 5].max
        row = lambda do |name, lh, lf, fh, ff|
          format("%-#{width}s  %5d/%-5d %s  %4d/%-4d %s", name, lh, lf, Lcov.pct(lh, lf), fh, ff, Lcov.pct(fh, ff))
        end
        rows = [format("%-#{width}s  %15s  %13s", "file", "lines", "functions")]
        files.sort.each do |name, f|
          rows << row.(name, f.lines_hit, f.lines_found, f.functions_hit, f.functions_found)
        end
        rows << row.("total", *totals.to_a)
        rows.join("\n")
      end

      # Write the merged coverage as an lcov tracefile.
      def write_lcov(path)
        out = ["TN:"]
        files.sort.each do |name, f|
          funcs = f.functions.sort_by { |fn, (ln, _)| [ln, fn] }
          out << "SF:#{name}"
          out.concat(funcs.map { |fn, (ln, _)| "FN:#{ln},#{fn}" })
          out.concat(funcs.map { |fn, (_, n)| "FNDA:#{n},#{fn}" })
          out.push("FNF:#{funcs.size}", "FNH:#{f.functions_hit}")
          out.concat(f.lines.sort.map { |ln, n| "DA:#{ln},#{n}" })
          out.push("LF:#{f.lines_found}", "LH:#{f.lines_hit}", "end_of_record")
        end
        File.write(path, "#{out.join("\n")}\n")
      end

      # Write an HTML report into `dir`: index.html, and a page per file with its source (read
      # from `root` for relative paths) colored by coverage.
      def write_html(dir, root: ".")
        FileUtils.mkdir_p(dir)
        rows = files.sort.each_with_index.map do |(name, f), i|
          link = "f#{i}.html"
          File.write(File.join(dir, link), Html.file_page(name, f, source_path(name, root)))
          Html.index_row(name, f, link)
        end
        File.write(File.join(dir, "index.html"), Html.index_page(totals, rows))
      end

      private

      def source_path(name, root) = Pathname(root.to_s).join(name).to_s
    end

    # The HTML report's pages.
    module Html
      CSS = <<~CSS
        body { font: 14px/1.4 -apple-system, system-ui, sans-serif; margin: 16px; color: #1d1d1f; background: #fff; }
        :root { color-scheme: light dark; }
        table { border-collapse: collapse; }
        td, th { padding: 2px 10px; text-align: right; }
        td:first-child, th:first-child { text-align: left; }
        tr:nth-child(even) { background: #f4f4f6; }
        a { color: #0550ae; } a:visited { color: #6639ba; }
        .bar { display: inline-block; width: 80px; height: 8px; background: #e5484d; vertical-align: middle; }
        .bar i { display: block; height: 100%; background: #30a46c; }
        pre { margin: 0; font: 12px/1.45 ui-monospace, Menlo, monospace; }
        .src td { padding: 0 8px; text-align: left; white-space: pre; font: 12px/1.45 ui-monospace, Menlo, monospace; }
        .src td.n { text-align: right; color: #888; user-select: none; }
        .hit { background: #d8f5e3; } .miss { background: #fbdcdc; }
        @media (prefers-color-scheme: dark) {
          body { color: #e8e8ea; background: #1b1b1f; } tr:nth-child(even) { background: #25252a; }
          .hit { background: #1d3b2a; } .miss { background: #4a2126; }
          a { color: #79b8ff; } a:visited { color: #c9a7ff; }
        }
      CSS
      ESCAPES = { "&" => "&amp;", "<" => "&lt;", ">" => "&gt;", '"' => "&quot;", "'" => "&#x27;" }.freeze

      module_function

      def h(text) = text.to_s.gsub(/[&<>"']/, ESCAPES)

      def page(title, body)
        "<!doctype html><html><head><meta charset='utf-8'><title>#{h(title)}</title>" \
          "<meta name='viewport' content='width=device-width'><style>\n#{CSS}</style></head>" \
          "<body>#{body}</body></html>\n"
      end

      def bar(hit, found)
        width = found.positive? ? 100.0 * hit / found : 0
        format("<span class='bar'><i style='width:%.0f%%'></i></span>", width)
      end

      def index_row(name, file, link)
        lh = file.lines_hit
        lf = file.lines_found
        fh = file.functions_hit
        ff = file.functions_found
        "<tr><td><a href='#{link}'>#{h(name)}</a></td><td>#{bar(lh, lf)}</td><td>#{Lcov.pct(lh, lf)}</td>" \
          "<td>#{lh}/#{lf}</td><td>#{Lcov.pct(fh, ff)}</td><td>#{fh}/#{ff}</td></tr>"
      end

      def index_page(totals, rows)
        lh, lf, fh, ff = totals.to_a
        page("Firmware coverage",
             "<h2>Firmware coverage</h2><p>lines #{lh}/#{lf} #{Lcov.pct(lh, lf)}, functions #{fh}/#{ff} " \
             "#{Lcov.pct(fh, ff)}</p><table><tr><th>file</th><th></th><th>lines</th><th></th><th>functions</th>" \
             "<th></th></tr>#{rows.join}</table>")
      end

      # A text's lines, split where Python's str.splitlines splits them (the report's line numbers
      # follow the compiler's, which counts form feeds and the like as line breaks too).
      def lines_of(text)
        lines = text.split(/\r\n|[\n\r\v\f\x1c\x1d\x1e\u0085\u2028\u2029]/, -1)
        lines.pop if lines.last == ""
        lines
      end

      def file_page(name, file, source)
        text = File.exist?(source) ? lines_of(Lcov.read_text(source)) : []
        count = [text.size, *file.lines.keys].max
        lines = (1..count).map do |n|
          hits = file.lines[n]
          cls = if hits.nil? then ""
                elsif hits.zero? then " class='miss'"
                else
                  " class='hit'"
                end
          "<tr#{cls}><td class='n'>#{n}</td><td class='n'>#{hits}</td><td>#{h(text[n - 1])}</td></tr>"
        end
        # (by line; functions on the same line keep their order)
        funcs = file.functions.each_with_index.sort_by { |(_, (ln, _)), i| [ln, i] }.map do |(fn, (ln, n)), _|
          "<tr><td>#{h(fn)}</td><td>#{ln}</td><td>#{n}</td></tr>"
        end
        missing = text.empty? ? "<p>Source not found at #{h(source)}.</p>" : ""
        page(name, "<p><a href='index.html'>index</a></p><h2>#{h(name)}</h2>" \
                   "<p>lines #{file.lines_hit}/#{file.lines_found} #{Lcov.pct(file.lines_hit, file.lines_found)}, " \
                   "functions #{file.functions_hit}/#{file.functions_found}</p>" \
                   "<table><tr><th>function</th><th>line</th><th>runs</th></tr>#{funcs.join}</table><br>#{missing}" \
                   "<table class='src'>#{lines.join}</table>")
      end
    end

    # The command line: inputs, then -o FILE, --include PREFIX
    # (repeatable), --html DIR, --root DIR, -q. Returns the exit status.
    def main(argv, out: $stdout, err: $stderr)
      opts = { include: [], root: "." }
      inputs = OptionParser.new do |o|
        o.banner = "coverage INPUTS... [options]: merge lcov tracefiles (or directories of them) and report"
        o.on("-o", "--output FILE", "write the merged tracefile here") { opts[:output] = _1 }
        o.on("--include PREFIX", "only files starting with PREFIX") { opts[:include] << _1 }
        o.on("--html DIR", "write an HTML report here") { opts[:html] = _1 }
        o.on("--root DIR", "where relative source paths are (for --html)") { opts[:root] = _1 }
        o.on("-q", "--quiet", "print only the total") { opts[:quiet] = true }
      end.parse(argv)
      report = Report.load(inputs, include: opts[:include], exclude: opts[:output])
      if report.tracefiles.zero?
        err.puts "no tracefiles found"
        return 1
      end
      report.write_lcov(opts[:output]) if opts[:output]
      report.write_html(opts[:html], root: opts[:root]) if opts[:html]
      out.puts(opts[:quiet] ? report.total_line : "#{report.summary}\n(#{report.tracefiles} tracefiles)")
      0
    end
  end
end
