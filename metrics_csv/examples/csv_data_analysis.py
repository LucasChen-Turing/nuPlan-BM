#!/usr/bin/env python3
"""
CSV Data Analysis Script

Script to analyze CSV planner data structure and generate mapping recommendations
for integration with nuPlan evaluation system.

Usage:
    python analyze_csv_data.py <csv_file_path> [options]

Author: Lucas, C., Claude Agent
Date: August 2025
"""

import argparse
import sys
from pathlib import Path

# Add parent directory to path for imports
sys.path.append(str(Path(__file__).parent.parent))

from planning.csv_data_analyzer import CSVDataAnalyzer


def main():
    """Main function for CSV data analysis."""
    parser = argparse.ArgumentParser(
        description='Analyze CSV planner data structure for nuPlan integration',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    # Basic analysis
    python analyze_csv_data.py planner_data.csv
    
    # Analysis with custom separator
    python analyze_csv_data.py planner_data.csv --sep ";"
    
    # Save analysis to specific file
    python analyze_csv_data.py planner_data.csv --output analysis_results.json
    
    # Skip header row
    python analyze_csv_data.py planner_data.csv --skiprows 1
        """
    )
    
    parser.add_argument(
        'csv_file',
        type=str,
        help='Path to CSV file containing planner data'
    )
    
    parser.add_argument(
        '--output', '-o',
        type=str,
        default=None,
        help='Output path for analysis results (default: auto-generated)'
    )
    
    parser.add_argument(
        '--sep',
        type=str,
        default=',',
        help='CSV separator (default: comma)'
    )
    
    parser.add_argument(
        '--skiprows',
        type=int,
        default=None,
        help='Number of rows to skip at beginning of file'
    )
    
    parser.add_argument(
        '--nrows',
        type=int,
        default=None,
        help='Number of rows to read (for large files)'
    )
    
    parser.add_argument(
        '--no-summary',
        action='store_true',
        help='Skip printing analysis summary'
    )
    
    parser.add_argument(
        '--verbose', '-v',
        action='store_true',
        help='Verbose output'
    )
    
    args = parser.parse_args()
    
    # Validate input file
    csv_path = Path(args.csv_file)
    if not csv_path.exists():
        print(f"Error: CSV file not found: {csv_path}")
        sys.exit(1)
    
    if not csv_path.is_file():
        print(f"Error: Path is not a file: {csv_path}")
        sys.exit(1)
    
    print(f"📂 Analyzing CSV file: {csv_path}")
    if args.verbose:
        print(f"   Separator: '{args.sep}'")
        print(f"   Skip rows: {args.skiprows}")
        print(f"   Max rows: {args.nrows}")
    
    try:
        # Initialize analyzer
        analyzer = CSVDataAnalyzer(str(csv_path))
        
        # Prepare CSV loading parameters
        load_params = {
            'sep': args.sep
        }
        
        if args.skiprows is not None:
            load_params['skiprows'] = args.skiprows
        
        if args.nrows is not None:
            load_params['nrows'] = args.nrows
        
        # Load CSV
        print("🔄 Loading CSV data...")
        if not analyzer.load_csv(**load_params):
            print("❌ Failed to load CSV file")
            sys.exit(1)
        
        # Perform analysis
        print("🔍 Analyzing data structure...")
        analysis_results = analyzer.analyze_structure()
        
        # Generate mapping recommendations
        print("🗺️  Generating mapping recommendations...")
        mapping = analyzer.generate_mapping_recommendations()
        
        # Print summary unless skipped
        if not args.no_summary:
            analyzer.print_summary()
        
        # Export results
        print("💾 Exporting analysis results...")
        output_path = analyzer.export_analysis(args.output)
        
        print(f"✅ Analysis complete!")
        print(f"📁 Results saved to: {output_path}")
        
        # Print quick recommendations
        print("\n🚀 QUICK INTEGRATION GUIDE:")
        print("=" * 50)
        
        if mapping.timestamp_col:
            print(f"✓ Time column detected: {mapping.timestamp_col}")
        else:
            print("⚠️  No timestamp column detected - you may need manual time alignment")
        
        if mapping.scenario_id_col:
            print(f"✓ Scenario ID column detected: {mapping.scenario_id_col}")
        else:
            print("⚠️  No scenario ID column detected - you'll need scenario matching strategy")
        
        essential_cols = [
            ('position_x', mapping.x_col),
            ('position_y', mapping.y_col),
            ('heading', mapping.heading_col)
        ]
        
        missing_essential = []
        for name, col in essential_cols:
            if col:
                print(f"✓ {name}: {col}")
            else:
                missing_essential.append(name)
        
        if missing_essential:
            print(f"❌ Missing essential columns: {', '.join(missing_essential)}")
            print("   These are required for trajectory creation!")
        
        if mapping.horizon_prefix and mapping.horizon_length:
            print(f"✓ Horizon data detected: {mapping.horizon_prefix}_* ({mapping.horizon_length} points)")
        else:
            print("⚠️  No horizon data detected - check if your CSV contains future trajectory points")
        
        print("\n📋 NEXT STEPS:")
        print("1. Review the analysis results JSON file")
        print("2. Verify the column mappings are correct")
        print("3. Run the integration test script")
        print("4. Create CSV-based evaluation pipeline")
        
    except Exception as e:
        print(f"❌ Error during analysis: {e}")
        if args.verbose:
            import traceback
            traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()