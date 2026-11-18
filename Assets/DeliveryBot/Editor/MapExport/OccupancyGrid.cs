using System;
using System.Globalization;
using System.IO;
using System.Text;

namespace DeliveryBot.MapExport
{
    /// <summary>
    /// 2D occupancy grid in the ROS map frame, written in our own map format
    /// (delivery_bot_core#12): <c>map.pgm</c> (8-bit, 0 = occupied, 254 = free, 205 = unknown) plus
    /// <c>map.yaml</c> (resolution, origin [x, y, yaw], occupied_thresh, free_thresh, negate: 0).
    ///
    /// Cell (col, row) covers x ∈ [OriginX + col·res, OriginX + (col+1)·res) and likewise for y with row.
    /// Row 0 is the lowest y. In the PGM, row 0 is the top image line, so rows are flipped on write.
    /// The origin is always a whole multiple of the resolution, so the map frame origin (= Unity world
    /// origin) lies exactly on a cell corner.
    /// </summary>
    public sealed class OccupancyGrid
    {
        public const byte Occupied = 0;
        public const byte Free = 254;
        public const byte Unknown = 205;

        // p = (255 - v) / 255 with negate 0: 0 → 1.0 (occupied), 205 → 0.196 (unknown), 254 → 0.004 (free).
        // free_thresh sits clearly between free and unknown so that 205 never reads as free.
        public const double OccupiedThresh = 0.65;
        public const double FreeThresh = 0.1;

        public readonly int Width;
        public readonly int Height;
        public readonly double Resolution;
        /// <summary>Index of cell (0, 0) in whole cells from the map origin. OriginX = CellOffsetX · Resolution.</summary>
        public readonly long CellOffsetX;
        public readonly long CellOffsetY;
        /// <summary>Row-major, row 0 = lowest y.</summary>
        public readonly byte[] Cells;

        public double OriginX => CellOffsetX * Resolution;
        public double OriginY => CellOffsetY * Resolution;

        public OccupancyGrid(int width, int height, double resolution, long cellOffsetX, long cellOffsetY, byte fill = Unknown)
        {
            if (width <= 0 || height <= 0) throw new ArgumentException("Grid must not be empty.");
            if (resolution <= 0) throw new ArgumentException("Resolution must be positive.");
            Width = width;
            Height = height;
            Resolution = resolution;
            CellOffsetX = cellOffsetX;
            CellOffsetY = cellOffsetY;
            Cells = new byte[width * height];
            if (fill != 0)
                for (int i = 0; i < Cells.Length; i++) Cells[i] = fill;
        }

        public byte this[int col, int row]
        {
            get => Cells[row * Width + col];
            set => Cells[row * Width + col] = value;
        }

        /// <summary>Global cell index of a coordinate (floor, with a small tolerance against round-off).</summary>
        public static long ToCellIndex(double coordinate, double resolution) =>
            (long)Math.Floor(coordinate / resolution + 1e-9);

        public bool TryGetCell(double x, double y, out int col, out int row)
        {
            col = (int)(ToCellIndex(x, Resolution) - CellOffsetX);
            row = (int)(ToCellIndex(y, Resolution) - CellOffsetY);
            return col >= 0 && row >= 0 && col < Width && row < Height;
        }

        public double CellCenterX(int col) => (CellOffsetX + col + 0.5) * Resolution;
        public double CellCenterY(int row) => (CellOffsetY + row + 0.5) * Resolution;

        public int Count(byte value)
        {
            int n = 0;
            foreach (byte c in Cells) if (c == value) n++;
            return n;
        }

        public OccupancyGrid Crop(int col0, int row0, int width, int height)
        {
            width = Math.Min(width, Width - col0);
            height = Math.Min(height, Height - row0);
            var tile = new OccupancyGrid(width, height, Resolution, CellOffsetX + col0, CellOffsetY + row0);
            for (int r = 0; r < height; r++)
                Array.Copy(Cells, (row0 + r) * Width + col0, tile.Cells, r * width, width);
            return tile;
        }

        // ---------------------------------------------------------------- file format

        public byte[] ToPgm()
        {
            byte[] header = Encoding.ASCII.GetBytes($"P5\n{Width} {Height}\n255\n");
            var bytes = new byte[header.Length + Cells.Length];
            Array.Copy(header, bytes, header.Length);
            for (int row = 0; row < Height; row++)
                Array.Copy(Cells, row * Width, bytes, header.Length + (Height - 1 - row) * Width, Width);
            return bytes;
        }

        public string ToYaml(string imageFile, string comment = null)
        {
            var sb = new StringBuilder();
            if (!string.IsNullOrEmpty(comment))
                foreach (string line in comment.Split('\n'))
                    sb.Append("# ").Append(line).Append('\n');
            sb.Append("image: ").Append(imageFile).Append('\n');
            sb.Append("resolution: ").Append(Num(Resolution)).Append('\n');
            sb.Append("origin: [").Append(Num(OriginX)).Append(", ").Append(Num(OriginY)).Append(", 0.0]\n");
            sb.Append("negate: 0\n");
            sb.Append("occupied_thresh: ").Append(Num(OccupiedThresh)).Append('\n');
            sb.Append("free_thresh: ").Append(Num(FreeThresh)).Append('\n');
            return sb.ToString();
        }

        /// <summary>Writes <paramref name="baseName"/>.pgm and <paramref name="baseName"/>.yaml into <paramref name="directory"/>.</summary>
        public void Save(string directory, string baseName, string comment = null)
        {
            Directory.CreateDirectory(directory);
            File.WriteAllBytes(Path.Combine(directory, baseName + ".pgm"), ToPgm());
            File.WriteAllText(Path.Combine(directory, baseName + ".yaml"), ToYaml(baseName + ".pgm", comment),
                new UTF8Encoding(false));
        }

        /// <summary>Reads a binary PGM (P5, maxval 255, no comments) back into image order (row 0 = top).</summary>
        public static byte[] ReadPgmPixels(byte[] pgm, out int width, out int height)
        {
            int pos = 0;
            string Token()
            {
                while (pos < pgm.Length && char.IsWhiteSpace((char)pgm[pos])) pos++;
                int start = pos;
                while (pos < pgm.Length && !char.IsWhiteSpace((char)pgm[pos])) pos++;
                return Encoding.ASCII.GetString(pgm, start, pos - start);
            }
            if (Token() != "P5") throw new FormatException("Not a binary PGM.");
            width = int.Parse(Token(), CultureInfo.InvariantCulture);
            height = int.Parse(Token(), CultureInfo.InvariantCulture);
            if (Token() != "255") throw new FormatException("Only maxval 255 is supported.");
            pos++; // single whitespace after maxval
            var pixels = new byte[width * height];
            Array.Copy(pgm, pos, pixels, 0, pixels.Length);
            return pixels;
        }

        static string Num(double v)
        {
            string s = v.ToString("0.0#########", CultureInfo.InvariantCulture);
            return s == "-0.0" ? "0.0" : s;
        }
    }
}
