#include <bits/stdc++.h>
#include "PID.h"
using namespace std;
template <class T> int size(const T &x) { return x.size(); }
#define rep(i,a,b) for (__typeof(a) i=(a); i<(b); ++i)
#define iter(it,c) for (__typeof((c).begin()) it = (c).begin(); it != (c).end(); ++it)
typedef pair<int, int> ii;
typedef vector<int> vi;
typedef vector<ii> vii;
typedef long long ll;
const int INF = ~(1<<31); // 2147483647

const double EPS = 1e-9;
const double pi = acos(-1);
typedef unsigned long long ull;
typedef vector<vi> vvi;
typedef vector<vii> vvii;
template <class T> T smod(T a, T b) { return (a % b + b) % b; }

int main(){
    cin.sync_with_stdio(false);
    PID myPID(23, 2, 0, 0.3, 0, 100, 1000);
    while(true){
        double inp;
        cin >> inp;
        cout << "Output is: " << myPID.Compute(inp) << endl;
    }
    return 0;
}