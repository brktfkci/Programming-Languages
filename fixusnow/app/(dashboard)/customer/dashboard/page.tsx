import { redirect } from "next/navigation";
import Link from "next/link";
import { auth } from "@/lib/auth";
import { prisma } from "@/lib/prisma";
import { formatCurrency, formatDate } from "@/lib/utils";
import { Plus } from "lucide-react";

const STATUS_COLORS: Record<string, string> = {
  OPEN: "bg-green-100 text-green-700",
  IN_PROGRESS: "bg-blue-100 text-blue-700",
  COMPLETED: "bg-gray-100 text-gray-600",
  CANCELLED: "bg-red-100 text-red-600",
};

export default async function CustomerDashboard() {
  const session = await auth();
  const user = session?.user as { id: string; role: string; name?: string } | undefined;

  if (!user) redirect("/login");
  if (user.role !== "CUSTOMER") redirect("/provider/dashboard");

  const tasks = await prisma.task.findMany({
    where: { customerId: user.id },
    include: {
      applications: { select: { id: true } },
      payment: { select: { status: true } },
    },
    orderBy: { createdAt: "desc" },
  });

  return (
    <div className="max-w-5xl mx-auto px-4 py-10">
      <div className="flex items-center justify-between mb-8">
        <div>
          <h1 className="text-3xl font-bold text-gray-900">My Tasks</h1>
          <p className="text-gray-500">Welcome back, {user.name}!</p>
        </div>
        <Link
          href="/customer/post-task"
          className="flex items-center gap-2 bg-blue-600 text-white px-5 py-2.5 rounded-xl font-semibold hover:bg-blue-700"
        >
          <Plus className="h-5 w-5" />
          New Task
        </Link>
      </div>

      {tasks.length === 0 ? (
        <div className="text-center py-20 bg-white rounded-2xl border border-gray-200">
          <p className="text-gray-400 text-lg mb-4">No tasks yet.</p>
          <Link
            href="/customer/post-task"
            className="bg-blue-600 text-white px-6 py-2.5 rounded-xl font-semibold hover:bg-blue-700"
          >
            Post Your First Task
          </Link>
        </div>
      ) : (
        <div className="space-y-4">
          {tasks.map((task: (typeof tasks)[number]) => (
            <Link key={task.id} href={`/tasks/${task.id}`}>
              <div className="bg-white border border-gray-200 rounded-xl p-5 hover:shadow-md hover:border-blue-300 transition-all flex items-center justify-between gap-4">
                <div className="flex-1 min-w-0">
                  <div className="flex items-center gap-2 mb-1">
                    <span className={`text-xs font-medium px-2 py-0.5 rounded-full ${STATUS_COLORS[task.status]}`}>
                      {task.status.replace("_", " ")}
                    </span>
                    <span className="text-xs text-gray-400">{task.category}</span>
                  </div>
                  <h3 className="font-semibold text-gray-900 truncate">{task.title}</h3>
                  <p className="text-sm text-gray-500">{formatDate(task.createdAt)}</p>
                </div>
                <div className="text-right shrink-0 space-y-1">
                  {task.budget && (
                    <p className="font-medium text-gray-700">{formatCurrency(task.budget)}</p>
                  )}
                  <p className="text-sm text-gray-500">
                    {task.applications.length} applicant{task.applications.length !== 1 ? "s" : ""}
                  </p>
                  {task.payment?.status === "PAID" && (
                    <span className="text-xs text-green-600 font-medium">Paid</span>
                  )}
                </div>
              </div>
            </Link>
          ))}
        </div>
      )}
    </div>
  );
}
