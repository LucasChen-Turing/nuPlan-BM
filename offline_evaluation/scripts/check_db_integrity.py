#!/usr/bin/env python3
"""
Check database integrity and find working databases
"""

import os
import sqlite3
from pathlib import Path

def check_database_integrity(db_path):
    """Check if database has complete schema"""
    try:
        conn = sqlite3.connect(db_path)
        cursor = conn.cursor()
        
        # Get all tables
        cursor.execute("SELECT name FROM sqlite_master WHERE type='table';")
        tables = [row[0] for row in cursor.fetchall()]
        
        # Required tables for nuPlan
        required_tables = ['ego_pose', 'scenario_tag', 'lidar_pc', 'track', 'traffic_light_status']
        
        missing_tables = [table for table in required_tables if table not in tables]
        has_complete_schema = len(missing_tables) == 0
        
        # Get scenario count if schema is complete
        scenario_count = 0
        scenario_tokens = []
        if has_complete_schema:
            try:
                cursor.execute("SELECT COUNT(*) FROM scenario_tag")
                scenario_count = cursor.fetchone()[0]
                
                cursor.execute("SELECT hex(token), type FROM scenario_tag LIMIT 5")
                scenario_tokens = cursor.fetchall()
            except Exception as e:
                print(f"Error getting scenario info: {e}")
        
        conn.close()
        
        return {
            'path': db_path,
            'has_complete_schema': has_complete_schema,
            'tables': tables,
            'missing_tables': missing_tables,
            'scenario_count': scenario_count,
            'scenario_tokens': scenario_tokens
        }
        
    except Exception as e:
        return {
            'path': db_path,
            'has_complete_schema': False,
            'error': str(e),
            'tables': [],
            'missing_tables': [],
            'scenario_count': 0,
            'scenario_tokens': []
        }

def main():
    """Check all databases in mini directory"""
    
    db_dir = Path("/home/chen/nuplan-devkit/data/cache/mini")
    db_files = list(db_dir.glob("*.db"))
    
    print(f"🔍 Checking {len(db_files)} database files for integrity...")
    
    working_databases = []
    corrupted_databases = []
    
    for db_file in sorted(db_files):
        print(f"\n=== {db_file.name} ===")
        
        result = check_database_integrity(db_file)
        
        if result['has_complete_schema']:
            print(f"✅ COMPLETE SCHEMA")
            print(f"   Tables: {len(result['tables'])}")
            print(f"   Scenarios: {result['scenario_count']}")
            if result['scenario_tokens']:
                print(f"   Sample tokens: {[token[0] for token in result['scenario_tokens'][:3]]}")
            working_databases.append(result)
        else:
            print(f"❌ INCOMPLETE SCHEMA")
            if 'error' in result:
                print(f"   Error: {result['error']}")
            else:
                print(f"   Missing: {result['missing_tables']}")
                print(f"   Has: {len(result['tables'])} tables")
            corrupted_databases.append(result)
    
    print(f"\n📊 SUMMARY:")
    print(f"   ✅ Working databases: {len(working_databases)}")
    print(f"   ❌ Corrupted databases: {len(corrupted_databases)}")
    
    if working_databases:
        print(f"\n🎯 RECOMMENDED WORKING DATABASES:")
        for db in working_databases[:5]:  # Show top 5
            print(f"   • {Path(db['path']).name}")
            print(f"     - Scenarios: {db['scenario_count']}")
            if db['scenario_tokens']:
                print(f"     - Sample token: {db['scenario_tokens'][0][0]}")
    
    # Create clean database directory with only working databases
    if working_databases:
        clean_dir = Path("/home/chen/nuplan-devkit/data/cache/mini_working")
        clean_dir.mkdir(exist_ok=True)
        
        print(f"\n🔧 Creating clean database directory: {clean_dir}")
        
        # Copy working databases
        import shutil
        for db in working_databases:
            src = Path(db['path'])
            dst = clean_dir / src.name
            if not dst.exists():
                shutil.copy2(src, dst)
                print(f"   ✓ Copied {src.name}")
        
        print(f"   📁 Clean directory created with {len(working_databases)} working databases")
        
        return str(clean_dir)
    
    return None

if __name__ == "__main__":
    clean_path = main()
    if clean_path:
        print(f"\n🎯 Next steps:")
        print(f"   1. Use data_root: {clean_path}")
        print(f"   2. Pick a scenario token from working databases")
        print(f"   3. Run nuPlan simulation")
    else:
        print(f"\n❌ No working databases found!")