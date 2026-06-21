import Link from "next/link";
import { Wrench, Shield, Star, Zap, ChevronRight } from "lucide-react";

const CATEGORIES = [
  { name: "Plumbing", icon: "🔧" },
  { name: "Electrical", icon: "⚡" },
  { name: "Cleaning", icon: "🧹" },
  { name: "Painting", icon: "🎨" },
  { name: "Carpentry", icon: "🪵" },
  { name: "Landscaping", icon: "🌿" },
  { name: "Moving", icon: "📦" },
  { name: "Handyman", icon: "🛠️" },
];

const HOW_IT_WORKS = [
  {
    step: "1",
    title: "Post Your Task",
    desc: "Describe what you need done, set your budget, and choose a date.",
  },
  {
    step: "2",
    title: "Receive Offers",
    desc: "Qualified local providers review your task and send you quotes.",
  },
  {
    step: "3",
    title: "Pick Your Provider",
    desc: "Compare profiles, reviews, and prices — then choose the best fit.",
  },
  {
    step: "4",
    title: "Get It Done",
    desc: "Pay securely through the platform. Leave a review when finished.",
  },
];

export default function HomePage() {
  return (
    <div>
      {/* Hero */}
      <section className="bg-gradient-to-br from-blue-600 to-blue-800 text-white py-24 px-4">
        <div className="max-w-4xl mx-auto text-center">
          <div className="flex justify-center mb-6">
            <Wrench className="h-14 w-14 text-blue-200" />
          </div>
          <h1 className="text-5xl font-bold mb-4">
            Fix Anything. <span className="text-blue-200">Fast.</span>
          </h1>
          <p className="text-xl text-blue-100 mb-10 max-w-2xl mx-auto">
            Post any home or local service task and get matched with trusted providers in your area
            — in minutes.
          </p>
          <div className="flex flex-col sm:flex-row gap-4 justify-center">
            <Link
              href="/signup?role=CUSTOMER"
              className="bg-white text-blue-700 font-semibold px-8 py-4 rounded-xl hover:bg-blue-50 transition text-lg"
            >
              Post a Task — Free
            </Link>
            <Link
              href="/signup?role=PROVIDER"
              className="border-2 border-white text-white font-semibold px-8 py-4 rounded-xl hover:bg-white/10 transition text-lg"
            >
              Become a Provider
            </Link>
          </div>
        </div>
      </section>

      {/* Trust badges */}
      <section className="bg-white border-b border-gray-100 py-6">
        <div className="max-w-5xl mx-auto px-4 flex flex-wrap justify-center gap-8 text-gray-600 text-sm font-medium">
          <span className="flex items-center gap-2"><Shield className="h-5 w-5 text-green-500" /> Verified Providers</span>
          <span className="flex items-center gap-2"><Star className="h-5 w-5 text-yellow-500" /> Rated &amp; Reviewed</span>
          <span className="flex items-center gap-2"><Zap className="h-5 w-5 text-blue-500" /> Fast Responses</span>
          <span className="flex items-center gap-2">💳 Secure Payments</span>
        </div>
      </section>

      {/* Categories */}
      <section className="py-16 px-4">
        <div className="max-w-5xl mx-auto">
          <h2 className="text-3xl font-bold text-gray-900 text-center mb-10">
            What do you need help with?
          </h2>
          <div className="grid grid-cols-2 sm:grid-cols-4 gap-4">
            {CATEGORIES.map((cat) => (
              <Link
                key={cat.name}
                href={`/tasks?category=${cat.name}`}
                className="bg-white border border-gray-200 rounded-xl p-5 text-center hover:border-blue-400 hover:shadow-md transition-all group"
              >
                <div className="text-4xl mb-2">{cat.icon}</div>
                <p className="font-medium text-gray-700 group-hover:text-blue-600">{cat.name}</p>
              </Link>
            ))}
          </div>
          <div className="text-center mt-8">
            <Link href="/tasks" className="text-blue-600 font-medium hover:underline flex items-center justify-center gap-1">
              View all open tasks <ChevronRight className="h-4 w-4" />
            </Link>
          </div>
        </div>
      </section>

      {/* How it works */}
      <section className="bg-white py-16 px-4">
        <div className="max-w-5xl mx-auto">
          <h2 className="text-3xl font-bold text-gray-900 text-center mb-12">How FixUsNow Works</h2>
          <div className="grid sm:grid-cols-2 lg:grid-cols-4 gap-8">
            {HOW_IT_WORKS.map((item) => (
              <div key={item.step} className="text-center">
                <div className="w-12 h-12 rounded-full bg-blue-600 text-white font-bold text-xl flex items-center justify-center mx-auto mb-4">
                  {item.step}
                </div>
                <h3 className="font-semibold text-gray-900 mb-2">{item.title}</h3>
                <p className="text-gray-500 text-sm leading-relaxed">{item.desc}</p>
              </div>
            ))}
          </div>
        </div>
      </section>

      {/* CTA */}
      <section className="py-20 px-4 bg-gradient-to-r from-blue-50 to-blue-100">
        <div className="max-w-2xl mx-auto text-center">
          <h2 className="text-3xl font-bold text-gray-900 mb-4">Ready to get started?</h2>
          <p className="text-gray-600 mb-8">
            Join hundreds of customers and providers in your city.
          </p>
          <Link
            href="/signup"
            className="bg-blue-600 text-white font-semibold px-10 py-4 rounded-xl hover:bg-blue-700 transition text-lg"
          >
            Create a Free Account
          </Link>
        </div>
      </section>
    </div>
  );
}
