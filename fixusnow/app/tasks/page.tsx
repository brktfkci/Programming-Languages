import { prisma } from "@/lib/prisma";
import { TaskCard } from "@/components/task-card";
import { SERVICE_CATEGORIES } from "@/lib/stripe";

interface Props {
  searchParams: Promise<{ category?: string; status?: string }>;
}

export default async function TasksPage({ searchParams }: Props) {
  const params = await searchParams;
  const category = params.category;
  const status = params.status || "OPEN";

  const tasks = await prisma.task.findMany({
    where: {
      ...(category ? { category } : {}),
      status: status as "OPEN" | "IN_PROGRESS" | "COMPLETED" | "CANCELLED",
    },
    include: {
      customer: { select: { id: true, name: true, image: true } },
      applications: { select: { id: true } },
    },
    orderBy: { createdAt: "desc" },
  });

  return (
    <div className="max-w-6xl mx-auto px-4 py-10">
      <h1 className="text-3xl font-bold text-gray-900 mb-2">Browse Tasks</h1>
      <p className="text-gray-500 mb-8">Find tasks in your area and submit an offer.</p>

      {/* Filters */}
      <div className="flex flex-wrap gap-2 mb-8">
        <a
          href="/tasks"
          className={`px-4 py-2 rounded-full text-sm font-medium border transition ${
            !category ? "bg-blue-600 text-white border-blue-600" : "border-gray-300 text-gray-600 hover:border-blue-400"
          }`}
        >
          All
        </a>
        {SERVICE_CATEGORIES.map((cat) => (
          <a
            key={cat}
            href={`/tasks?category=${cat}`}
            className={`px-4 py-2 rounded-full text-sm font-medium border transition ${
              category === cat
                ? "bg-blue-600 text-white border-blue-600"
                : "border-gray-300 text-gray-600 hover:border-blue-400"
            }`}
          >
            {cat}
          </a>
        ))}
      </div>

      {tasks.length === 0 ? (
        <div className="text-center py-20 text-gray-400">
          <p className="text-lg">No tasks found.</p>
          <p className="text-sm mt-1">Check back later or try a different category.</p>
        </div>
      ) : (
        <div className="grid sm:grid-cols-2 lg:grid-cols-3 gap-5">
          {tasks.map((task: (typeof tasks)[number]) => (
            <TaskCard
              key={task.id}
              task={{
                ...task,
                customer: { name: task.customer.name },
              }}
            />
          ))}
        </div>
      )}
    </div>
  );
}
