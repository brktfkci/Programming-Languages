import { redirect } from "next/navigation";
import Link from "next/link";
import { auth } from "@/lib/auth";
import { prisma } from "@/lib/prisma";
import { formatCurrency, formatDate } from "@/lib/utils";
import { Star } from "lucide-react";

const STATUS_COLORS: Record<string, string> = {
  PENDING: "bg-yellow-100 text-yellow-700",
  ACCEPTED: "bg-green-100 text-green-700",
  REJECTED: "bg-red-100 text-red-600",
};

export default async function ProviderDashboard() {
  const session = await auth();
  const user = session?.user as { id: string; role: string; name?: string } | undefined;

  if (!user) redirect("/login");
  if (user.role !== "PROVIDER") redirect("/customer/dashboard");

  const [applications, profile] = await Promise.all([
    prisma.application.findMany({
      where: { providerId: user.id },
      include: {
        task: {
          select: {
            id: true,
            title: true,
            category: true,
            status: true,
            city: true,
            createdAt: true,
          },
        },
      },
      orderBy: { createdAt: "desc" },
    }),
    prisma.providerProfile.findUnique({ where: { userId: user.id } }),
  ]);

  return (
    <div className="max-w-5xl mx-auto px-4 py-10">
      <div className="flex items-start justify-between mb-8">
        <div>
          <h1 className="text-3xl font-bold text-gray-900">Provider Dashboard</h1>
          <p className="text-gray-500">Welcome back, {user.name}!</p>
        </div>
        <Link
          href="/tasks"
          className="bg-blue-600 text-white px-5 py-2.5 rounded-xl font-semibold hover:bg-blue-700"
        >
          Browse Open Tasks
        </Link>
      </div>

      {/* Stats */}
      <div className="grid grid-cols-3 gap-4 mb-10">
        <div className="bg-white rounded-xl border border-gray-200 p-5 text-center">
          <p className="text-3xl font-bold text-blue-600">{applications.length}</p>
          <p className="text-gray-500 text-sm mt-1">Total Applications</p>
        </div>
        <div className="bg-white rounded-xl border border-gray-200 p-5 text-center">
          <p className="text-3xl font-bold text-green-600">{profile?.jobsDone ?? 0}</p>
          <p className="text-gray-500 text-sm mt-1">Jobs Completed</p>
        </div>
        <div className="bg-white rounded-xl border border-gray-200 p-5 text-center">
          <div className="flex items-center justify-center gap-1">
            <p className="text-3xl font-bold text-yellow-500">
              {profile?.rating ? profile.rating.toFixed(1) : "—"}
            </p>
            {profile?.rating ? <Star className="h-6 w-6 text-yellow-500" fill="currentColor" /> : null}
          </div>
          <p className="text-gray-500 text-sm mt-1">Rating</p>
        </div>
      </div>

      {/* Applications */}
      <h2 className="text-xl font-bold text-gray-900 mb-4">My Applications</h2>
      {applications.length === 0 ? (
        <div className="text-center py-16 bg-white rounded-2xl border border-gray-200">
          <p className="text-gray-400 text-lg mb-4">No applications yet.</p>
          <Link href="/tasks" className="bg-blue-600 text-white px-6 py-2.5 rounded-xl font-semibold hover:bg-blue-700">
            Find Tasks
          </Link>
        </div>
      ) : (
        <div className="space-y-4">
          {applications.map((app) => (
            <Link key={app.id} href={`/tasks/${app.task.id}`}>
              <div className="bg-white border border-gray-200 rounded-xl p-5 hover:shadow-md hover:border-blue-300 transition-all flex items-center justify-between gap-4">
                <div className="flex-1 min-w-0">
                  <div className="flex items-center gap-2 mb-1">
                    <span className={`text-xs font-medium px-2 py-0.5 rounded-full ${STATUS_COLORS[app.status]}`}>
                      {app.status}
                    </span>
                    <span className="text-xs text-gray-400">{app.task.category}</span>
                  </div>
                  <h3 className="font-semibold text-gray-900 truncate">{app.task.title}</h3>
                  <p className="text-sm text-gray-500">{app.task.city} · {formatDate(app.createdAt)}</p>
                </div>
                <div className="text-right shrink-0">
                  <p className="font-bold text-gray-900">{formatCurrency(app.price)}</p>
                  <p className="text-xs text-gray-400 capitalize">
                    Task: {app.task.status.replace("_", " ").toLowerCase()}
                  </p>
                </div>
              </div>
            </Link>
          ))}
        </div>
      )}
    </div>
  );
}
