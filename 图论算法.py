# 743 网络延迟时间
# Bellman - Ford算法
from queue import PriorityQueue as PQ
class NetworkDelayTime:
    def networkDelayTime(self,times,n,k):
        dist = [1e9] * (n+1)
        flag = False
        dist[k] = 0
        ans = 0
        for i in range(n-1):
            flag = False
            for each in times:
                x = each[0]
                y = each[1]
                z = each[2]
                if dist[y] > dist[x] + z:
                    dist[y] = dist[x] + z
                    flag = True
            if not flag:
                break
        for i in range(1,n+1):
            ans = max(ans,dist[i])
        if ans == 1e9:
            return -1
        else:
            return ans
# 迪科斯特拉算法
    def networkDelayTime2(self,times,n,k):
        dist = [1e9] * (n + 1)
        dist[k] = 0
        q = PQ()
        q.put((dist[k],k))
        # 建图
        visited = [False] * (n+1)
        edges = [[] for each in range(n+1)]
        for each in times:
            x = each[0]
            y = each[1]
            z = each[2]
            edges[x].append((y,z))
        while not q.empty():
            top = q.get()
            if visited[top[1]]:
                continue
            for each_edge in edges[top[1]]:
                if dist[each_edge[0]] > dist[top[1]] + each_edge[1]:
                    dist[each_edge[0]] = dist[top[1]] + each_edge[1]
                    q.put((dist[each_edge[0]],each_edge[0]))
        ans = 0
        for i in range(1,n+1):
            ans = max(ans,dist[i])
        if ans == 1e9:
            return -1
        else:
            return ans
# ACwing 850. Dijkstra求最短路 II
if __name__ == '__main__':
    n,m = map(int,input().split())
    dist = [1e9] * (n + 1)
    visited = [False] * (n + 1)
    edges = [[] for each in range(n+1)]
    for i in range(m):
        x,y,z = map(int,input().split())
        edges[x].append((y,z))
    dist[1] = 0
    q = PQ()
    q.put((dist[1],1))
    while not q.empty():
        top = q.get()
        if visited[top[1]]:
            continue
        visited[top[1]] = True
        for each_edge in edges[top[1]]:
            start,end,dis = top[1],each_edge[0],each_edge[1]
            if dist[end] > dist[start] + dis:
                dist[end] = dist[start] + dis
                q.put((dist[end],end))
    if dist[n] == 1e9:
        print(-1)
    else:
        print(dist[n])
# Floyd 算法
# 1334 阈值距离内邻居最少的城市
class FindTheCity:
    def findTheCity(self,n,edges,distanceThreshold):
        dist = [[1e9] * n for i in range(n)]
        for each_edge in edges:
            x,y,z = each_edge[0],each_edge[1],each_edge[2]
            dist[x][y] = dist[y][x] = z
        for i in range(n):
            dist[i][i] = 0
        for k in range(n):
            for i in range(n):
                for j in range(n):
                    dist[i][j] = min(dist[i][j],dist[i][k] + dist[k][j])
        ansMin = n
        ans = 0       
        for i in range(n):
            count = 0
            for j in range(n):
                if dist[i][j] <= distanceThreshold:
                    count += 1
            if count <= ansMin:
                ansMin = count
                ans = i
        return ans
# 最小生成树
# Kruskal 算法
# 1584. 连接所有点的最小费用
class MinCostConnectPoints:
    def minCostConnectPoints(self,points):
        edges = []
        n = len(points)
        ans = 0
        for i in range(n):
            for j in range(i+1,n):
                edges.append([i,j,abs(points[i][0] - points[j][0]) + abs(points[i][1] - points[j][1])])
        edges.sort(key = lambda x:x[2])
        self.fa = [i for i in range(n)]
        count = 0
        for each_edge in edges:
            x,y,z = each_edge[0],each_edge[1],each_edge[2]
            x = self.find(x)
            y = self.find(y)
            if x != y:
                self.fa[x] = y
                ans += z
                count += 1
            if count == n - 1:
                break
        return ans
    def find(self,x):
        if x == self.fa[x]:
            return x
        self.fa[x] = self.find(self.fa[x])
        return self.fa[x]



            

                


                
















